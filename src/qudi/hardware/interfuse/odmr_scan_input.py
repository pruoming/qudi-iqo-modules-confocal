# -*- coding: utf-8 -*-

"""
Qudi hardware module (confocal_odmr): FiniteSamplingInput that produces one CW-ODMR sweep line —
fluorescence vs microwave frequency — by coordinating the Pulse Streamer (master clock) and the
Time Tagger, while the SMIQ steps its frequency list on an EXTERNAL trigger.

It implements FiniteSamplingInputInterface so the stock qudi ODMR stack (OdmrGui + OdmrLogic) can be
reused UNCHANGED. OdmrLogic connects:
    microwave     -> mw_source_smiq   (MicrowaveInterface, already validated; RF gated by the cap)
    data_scanner  -> THIS module      (FiniteSamplingInputInterface)
Per sweep line OdmrLogic does:  configure_scan(power, frequencies, JUMP_LIST, sample_rate) +
start_scan()  (arms the SMIQ list, output ON, EXT-triggered, parked at the first entry), then calls
data_scanner.acquire_frame()  (this module: N frequency points), then microwave.reset_scan().

This is the CW-ODMR analogue of hardware/interfuse/confocal_scan_io.py, but SIMPLER:
  * it sweeps FREQUENCY, not position: there is NO NI AO / PFI0 external clock at all;
  * the per-point "advance" pulse goes to the SMIQ frequency-advance trigger (Pulse Streamer ch6 =
    'next', connections.yaml) instead of the NI-AO pixel clock;
  * ONE count window per point (mw is continuously ON in CW-ODMR), not the 2 mw-on/mw-off windows
    of the confocal diff scan.
Reused verbatim from the confocal bring-up (see Qudi_AI/setups/confocal_odmr/known_issues.yaml):
  * SCAN-006 ARM BARRIER: tagger.sync() + a short cbm_arm_delay BEFORE starting the Pulse Streamer,
    because a freshly created CountBetweenMarkers is not armed instantly (early 'detect' edges lost).
  * SCAN-002 progressive readout via CountBetweenMarkers.getBinWidths() (closed-window count works
    with the laser off, unlike counting nonzero photons).
  * SCAN-001 self-contained window: the detect pattern ends each point with a low 'tail' so every
    count window's closing edge falls inside its own point (else the last window never closes).
  * SCAN-007-style edge alignment: whether the SMIQ advance fires at the point START or END changes
    which frequency each window is measured at. Default advance_at_start=True (advance -> settle ->
    count). VERIFY EMPIRICALLY with a Phase-3 probe (RF off) that point i is measured at frequency i
    — the SMIQ list is written with its first entry DUPLICATED (mw_source_smiq._write_list), an
    external-trigger workaround that shifts the trigger/point correspondence.

Shared Time Tagger: like confocal_scan_io this module can OWN the Time Tagger (createTimeTagger) and
expose get_tagger() so a live counter (timetagger_instreamer) can borrow it; OR it can itself borrow
one from another owner via the optional tagger_provider connector. One connection, many measurements.

SAFETY / STATUS: FIRST DRAFT, CW-ODMR. This module gates the laser (ch0) and the mw switch (ch4) via
the Pulse Streamer, and pulses the SMIQ frequency-advance trigger (ch6). It does NOT set RF power or
enable the SMIQ RF output — OdmrLogic/mw_source_smiq own that, gated by the human-confirmed power cap
(Phase 0, still to_be_confirmed). Validate MOCK first (finite_sampling_input_dummy), then RF-OFF
(sweep + counting mechanics), then RF-ON only with the approved cap. Lines marked `# VALIDATE` need a
hardware check. Measurement correctness needs expert validation. Codex review advised.

This file is part of qudi. Licensed under LGPL v3.
"""

import time
import numpy as np

import pulsestreamer as ps
import TimeTagger as tt

from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector
from qudi.util.mutex import Mutex
from qudi.interface.finite_sampling_input_interface import (FiniteSamplingInputInterface,
                                                            FiniteSamplingInputConstraints)


class OdmrScanInput(FiniteSamplingInputInterface):
    """
    Example config:

    odmr_scan_input:
        module.Class: 'interfuse.odmr_scan_input.OdmrScanInput'
        options:
            pulsestreamer_ip: '169.254.8.2'
            ps_channels:                 # Pulse Streamer digital channels (connections.yaml)
                laser: 0
                mw: 4                    # mw switch gate (ZASWA-2-50DR+)
                detect: 1                # count-gate marker -> Time Tagger ch5
                next: 6                  # SMIQ frequency-advance trigger (rising edge)
            timetagger_serial: ''        # optional; '' -> the only Time Tagger
            apd_channels: [1, 2]         # Time Tagger click channels (APD1, APD2)
            detect_tt_channel: 5         # Time Tagger channel receiving the 'detect' marker
            input_channel_name: 'fluorescence'
            mw_settle_time: 1.0e-3       # s, dwell AFTER the SMIQ advance before counting.
                                         # to_be_confirmed vs the SMIQ settle spec (~few ms). TUNE.
            accumulate_fraction: 0.9     # fraction of the post-settle window actually counted
            next_pulse_ns: 100           # width of the SMIQ-advance pulse (ch6)
            advance_at_start: True        # advance at point START (advance->settle->count); SCAN-007
            cw_mw_on: True               # hold the mw switch (ch4) OPEN for the whole line (CW-ODMR)
            enable_laser: False          # True drives ch0 laser HIGH during the line -- HUMAN APPROVAL
            cbm_arm_delay: 0.05          # SCAN-006 arm barrier (s) before starting the Pulse Streamer
            default_sample_rate: 200.0   # Hz -> 5 ms/point (OdmrLogic overrides via set_sample_rate)
            # frame_timeout: null        # s; null -> auto (expected frame time x2 + 5 s)
        # connect:                       # OPTIONAL: borrow a Time Tagger already owned elsewhere
        #     tagger_provider: <owner_with_get_tagger>
    """
    # Required only when this module opens its OWN Pulse Streamer (no ps_provider connected).
    _ps_ip = ConfigOption('pulsestreamer_ip', default=None)
    _ps_channels = ConfigOption('ps_channels', missing='error')
    _tt_serial = ConfigOption('timetagger_serial', default='')
    _apd_channels = ConfigOption('apd_channels', missing='error')
    _detect_tt_channel = ConfigOption('detect_tt_channel', default=5)
    _input_channel_name = ConfigOption('input_channel_name', default='fluorescence')
    # Dwell AFTER the SMIQ frequency-advance trigger and BEFORE the count window opens, so photons
    # are integrated only once the SMIQ has settled at the new frequency. The SMIQ is slow to switch
    # (~few ms, measurement_techniques.md); counting during the transit smears fluorescence across
    # two frequencies. HARDWARE-SPECIFIC and to_be_confirmed vs the SMIQ settle spec -- TUNE against
    # the device (raise until the dip stops depending on sweep direction). Shrinks the count window,
    # so lower default_sample_rate if you need more count time.
    _mw_settle_time = ConfigOption('mw_settle_time', default=1.0e-3)
    _accumulate_fraction = ConfigOption('accumulate_fraction', default=0.9)
    _next_pulse_ns = ConfigOption('next_pulse_ns', default=100)
    # Advance the SMIQ at the START of each point (advance -> settle -> count) rather than the end.
    # Which frequency each window is measured at depends on this AND on the SMIQ list's duplicated
    # first entry (mw_source_smiq._write_list). VERIFY EMPIRICALLY (Phase 3 probe, RF off). SCAN-007.
    _advance_at_start = ConfigOption('advance_at_start', default=True)
    # CW-ODMR: hold the mw switch (ch4) OPEN for the whole line so RF passes continuously. Set False
    # only for a pulsed variant (not this module). The SMIQ RF output itself is enabled by OdmrLogic
    # (start_scan), gated by the human-confirmed power cap -- this only opens/closes the downstream
    # switch.
    _cw_mw_on = ConfigOption('cw_mw_on', default=True)
    # 'sum'  -> ONE count window per point, mw switch (ch4) held OPEN the whole point (plain CW-ODMR):
    #           output = PL count rate (this is what shows the fluorescence, incl. any baseline drift).
    # 'diff' -> TWO windows per point: A with mw ON, then B with mw OFF; output = rate(A) - rate(B).
    #           A and B are taken back-to-back at the SAME frequency, so a slowly-drifting PL baseline
    #           (e.g. bleaching under illumination) is COMMON to both and cancels, leaving only the
    #           mw-induced change -> a flat ~0 baseline with the resonance as a dip. The SMIQ RF must be
    #           ON (as always for ODMR); the mw switch does the fast per-point on/off gating. Diff shows
    #           ~0 everywhere until real RF reaches the sample (mw on/off then makes no PL difference).
    _scan_mode = ConfigOption('scan_mode', default='sum')
    _enable_laser = ConfigOption('enable_laser', default=False)
    # Hold the laser ON continuously between sweep lines and after the scan (not just during each
    # streamed line), so the PL source sees STEADY illumination instead of the laser blinking off
    # between lines (which restarts bleaching transients and distorts the baseline). Only acts when
    # enable_laser is True. Turn it off with laser_off(); the shared Pulse Streamer is also returned to
    # LOW by its provider on qudi shutdown. (Operator request 2026-07-02.)
    _keep_laser_on = ConfigOption('keep_laser_on', default=True)
    # SCAN-006 arm barrier: a freshly created CountBetweenMarkers needs ~tens of ms to start
    # listening; without this the first detect edges are lost -> count<->point registration slip.
    _cbm_arm_delay = ConfigOption('cbm_arm_delay', default=0.05)
    _default_sample_rate = ConfigOption('default_sample_rate', default=200.0)
    # safety net: max seconds to wait for a Time Tagger frame before aborting (so a missing detect
    # marker or a stalled SMIQ can't block the sweep forever). None -> auto (expected time x2 + 5 s).
    _frame_timeout = ConfigOption('frame_timeout', default=None)

    # OPTIONAL: borrow the Time Tagger from a dedicated owner module (timetagger_provider), so several
    # modules share ONE connection. If not connected, this module opens its own (standalone ODMR use).
    _tagger_provider = Connector(name='tagger_provider', interface='TimeTaggerProvider', optional=True)
    # OPTIONAL: borrow the Pulse Streamer from a dedicated owner module (pulsestreamer_provider) so a
    # combined confocal+ODMR config drives ONE Pulse Streamer connection rather than two independent
    # ones. If not connected, this module opens its own (needs pulsestreamer_ip).
    _ps_provider = Connector(name='ps_provider', interface='PulseStreamerProvider', optional=True)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = Mutex()
        self._tagger = None
        self._owns_tagger = False
        self._pulser = None
        self._owns_pulser = False
        self._constraints = None
        # active sweep state
        self._sample_rate = 0.0
        self._frame_size = 0
        self._active_channels = frozenset()
        self._cbm = None                    # TimeTagger.CountBetweenMarkers
        self._combiner = None
        self._detect_monitor = None         # TimeTagger.Countrate on the detect channel (diagnostic)
        self._consumed_points = 0
        self._frame_start_time = None
        self._windows_per_point = 1          # 1 (sum) or 2 (diff: window A mw-on, B mw-off)
        self._win_a_time = None              # window A integration time (s), set when seq is built
        self._win_b_time = None              # window B integration time (s); diff mode only

    # ---------------------------------------------------------------- lifecycle
    def on_activate(self):
        # Validate timing-critical config up front so a typo fails fast with a clear message rather
        # than producing negative Pulse Streamer segment durations or a late error mid-sweep.
        if not (0.0 < float(self._accumulate_fraction) < 1.0):
            raise ValueError(f'accumulate_fraction must be in (0, 1), got {self._accumulate_fraction}')
        if int(self._next_pulse_ns) <= 0:
            raise ValueError(f'next_pulse_ns must be positive, got {self._next_pulse_ns}')
        if str(self._scan_mode).lower() not in ('sum', 'diff'):
            raise ValueError(f"scan_mode must be 'sum' or 'diff', got {self._scan_mode}")
        for _name in ('_mw_settle_time', '_cbm_arm_delay'):
            _val = getattr(self, _name)
            if _val is not None and float(_val) < 0:
                raise ValueError(f'{_name[1:]} must be >= 0, got {_val}')
        for _key in ('laser', 'mw', 'detect', 'next'):
            if _key not in self._ps_channels:
                raise ValueError(f"ps_channels missing required key '{_key}'")

        # Use a shared Pulse Streamer if a provider is connected (combined confocal+ODMR config), else
        # open our own. When borrowing, DON'T force the outputs low here — the owner (e.g.
        # confocal_scan_io) manages the idle state; only our own connection starts at ZERO.
        ps_owner = None
        try:
            ps_owner = self._ps_provider()
        except Exception:
            ps_owner = None
        if ps_owner is not None:
            self._pulser = ps_owner.get_pulser()
            self._owns_pulser = False
            self.log.info('Using shared Pulse Streamer from the ps_provider module.')
        else:
            if not self._ps_ip:
                raise ValueError('pulsestreamer_ip is required when no ps_provider connector is set')
            self._pulser = ps.PulseStreamer(self._ps_ip)
            self._owns_pulser = True
            self._pulser.constant(ps.OutputState.ZERO())  # all outputs LOW (laser/mw OFF, no advance)

        # Use a shared Time Tagger if a provider module is connected, else open our own.
        provider = None
        try:
            provider = self._tagger_provider()
        except Exception:
            provider = None
        if provider is not None:
            self._tagger = provider.get_tagger()
            self._owns_tagger = False
            self.log.info(f'Using shared Time Tagger, serial {self._tagger.getSerial()}.')
        else:
            self._tagger = tt.createTimeTagger(self._tt_serial) if self._tt_serial \
                else tt.createTimeTagger()
            self._owns_tagger = True
            self.log.info(f'Connected own Time Tagger, serial {self._tagger.getSerial()}.')

        self._constraints = FiniteSamplingInputConstraints(
            channel_units={self._input_channel_name: 'c/s'},
            frame_size_limits=(1, int(1e7)),
            sample_rate_limits=(0.1, 1e4),  # VALIDATE vs Pulse Streamer / SMIQ settle bandwidth
        )
        self._sample_rate = float(self._default_sample_rate)
        self._frame_size = 0
        self._active_channels = frozenset(self._constraints.channel_names)

    def on_deactivate(self):
        try:
            self.stop_buffered_acquisition()
        except Exception:
            pass
        # Force outputs LOW only if we OWN the Pulse Streamer; a borrowed one is managed by its owner
        # (stop_buffered_acquisition above already returned a running sweep's outputs to ZERO).
        if self._owns_pulser and self._pulser is not None:
            try:
                self._pulser.constant(ps.OutputState.ZERO())  # leave all outputs LOW (safe)
            except Exception:
                pass
        # Only free the device if we opened it ourselves (a borrowed tagger is owned elsewhere).
        if self._owns_tagger and self._tagger is not None:
            try:
                tt.freeTimeTagger(self._tagger)
            except Exception:
                pass
        self._tagger = None
        self._pulser = None

    # ---------------------------------------------------------------- properties
    @property
    def constraints(self):
        return self._constraints

    @property
    def active_channels(self):
        return frozenset(self._active_channels)

    @property
    def sample_rate(self):
        return self._sample_rate

    @property
    def frame_size(self):
        return self._frame_size

    @property
    def samples_in_buffer(self):
        # Points whose count window has closed and not yet been consumed.
        if self.module_state() != 'locked' or self._cbm is None:
            return 0
        return max(0, self._available_points_total() - self._consumed_points)

    # ---------------------------------------------------------------- configuration
    def set_sample_rate(self, rate):
        lo, hi = self._constraints.sample_rate_limits
        if not (lo <= float(rate) <= hi):
            raise ValueError(f'Sample rate {rate} out of bounds {(lo, hi)}')
        with self._lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Unable to set sample rate. Acquisition in progress.')
            self._sample_rate = float(rate)

    def set_active_channels(self, channels):
        chnl_set = frozenset(channels)
        if not chnl_set.issubset(self._constraints.channel_names):
            raise ValueError(f'Invalid channels {channels}; allowed {self._constraints.channel_names}')
        with self._lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Unable to set active channels. Acquisition in progress.')
            self._active_channels = chnl_set

    def set_frame_size(self, size):
        samples = int(round(size))
        if not self._constraints.frame_size_in_range(samples)[0]:
            raise ValueError(f'frame size {samples} out of bounds {self._constraints.frame_size_limits}')
        with self._lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Unable to set frame size. Acquisition in progress.')
            self._frame_size = samples

    # ---------------------------------------------------------------- Pulse Streamer sequence
    def _build_point_sequence(self):
        """ One frequency-point block (duration T = 1/sample_rate), streamed frame_size times.

        Layout (advance_at_start=True): [next pulse ch6 -> advance SMIQ][settle][count region][tail].
        (advance_at_start=False puts the next pulse at the END; 'lead' offset changes accordingly.)
        sum mode  -> ONE count window over the whole count region; mw switch (ch4) held OPEN (CW).
        diff mode -> TWO windows: A (mw ON) then B (mw OFF), split by a short 'skip' dead-gap that also
                     lets the mw switch settle; mw (ch4) is HIGH over window A only. Output = A - B.
        Each count window is bracketed by a detect rising (start) + falling (end) edge (ch1 -> TT ch5);
        the trailing 'tail' low guarantees the last window closes inside the point (SCAN-001). 'laser'
        (ch0) is HIGH for the whole point iff enable_laser.
        """
        T_ns = int(round(1e9 / self._sample_rate))
        pw = int(min(self._next_pulse_ns, max(1, T_ns // 100)))
        settle = int(round(float(self._mw_settle_time) * 1e9))
        tail = pw
        diff = str(self._scan_mode).lower() == 'diff'
        ch = self._ps_channels
        lead = (pw + settle) if self._advance_at_start else settle   # offset to the count region
        L = T_ns - pw - settle - tail                                # count-region length
        if L <= 0:
            raise RuntimeError(
                f'Invalid point timing: mw_settle_time={self._mw_settle_time} s + pulses leave no '
                f'count region at {self._sample_rate} Hz (point {T_ns} ns). Lower mw_settle_time or '
                f'the sample rate.')

        def _seg(pairs):
            return [(int(d), int(v)) for (d, v) in pairs if int(d) > 0]  # drop zero-length segments

        def _pad(pattern):
            s = sum(int(d) for d, _ in pattern)
            return pattern + [(T_ns - s, 0)] if s < T_ns else pattern

        seq = self._pulser.createSequence()
        # SMIQ frequency-advance pulse (ch6): at the point START (default) or END (legacy)
        if self._advance_at_start:
            seq.setDigital(ch['next'], _seg([(pw, 1), (T_ns - pw, 0)]))
        else:
            seq.setDigital(ch['next'], _seg([(T_ns - pw, 0), (pw, 1)]))

        if not diff:
            skip = int(round((1.0 - float(self._accumulate_fraction)) * L))
            count_win = L - skip
            if count_win <= 0:
                raise RuntimeError(f'Invalid point timing: count window {count_win} ns at '
                                   f'{self._sample_rate} Hz. Lower mw_settle_time or sample rate.')
            self._win_a_time = count_win / 1e9
            self._win_b_time = None
            seq.setDigital(ch['detect'], _seg(_pad([(lead + skip, 0), (count_win, 1), (tail, 0)])))
            if self._cw_mw_on:
                seq.setDigital(ch['mw'], [(T_ns, 1)])       # mw switch OPEN for the whole point (CW)
        else:
            half = L // 2
            skip = int(round((1.0 - float(self._accumulate_fraction)) / 2.0 * L))
            win_a = half - skip
            win_b = L - half - skip
            if win_a <= 0 or win_b <= 0:
                raise RuntimeError(f'Invalid diff timing: window A={win_a} ns, B={win_b} ns at '
                                   f'{self._sample_rate} Hz. Lower mw_settle_time or the sample rate.')
            self._win_a_time = win_a / 1e9
            self._win_b_time = win_b / 1e9
            # detect: [lead+skip low][A high][skip low][B high][tail low] -> 2 windows/point
            seq.setDigital(ch['detect'], _seg(_pad([(lead + skip, 0), (win_a, 1),
                                                    (skip, 0), (win_b, 1), (tail, 0)])))
            # mw switch OPEN over window A region only ([lead, lead+half)), CLOSED over window B
            seq.setDigital(ch['mw'], _seg(_pad([(lead, 0), (half, 1)])))
        if self._enable_laser:
            seq.setDigital(ch['laser'], [(T_ns, 1)])    # laser HIGH -- HUMAN-APPROVED
        return seq

    # ---------------------------------------------------------------- acquisition control
    def start_buffered_acquisition(self):
        with self._lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Acquisition already running')
            if self._frame_size < 1:
                raise RuntimeError('No frame size set (need > 0 frequency points)')
            self.module_state.lock()
            try:
                n = self._frame_size
                self._windows_per_point = 2 if str(self._scan_mode).lower() == 'diff' else 1
                # Time Tagger: combine APDs, count between detect rising/falling edges. ONE window per
                # point in sum mode, TWO (mw-on A, mw-off B) in diff mode.
                self._combiner = tt.Combiner(self._tagger, channels=list(self._apd_channels))
                self._cbm = tt.CountBetweenMarkers(
                    self._tagger,
                    click_channel=self._combiner.getChannel(),
                    begin_channel=int(self._detect_tt_channel),
                    end_channel=-int(self._detect_tt_channel),   # falling edge of same channel
                    n_values=n * self._windows_per_point)
                # diagnostic: detect edges actually arriving at the Time Tagger during the frame
                self._detect_monitor = tt.Countrate(self._tagger, [int(self._detect_tt_channel)])
                self._consumed_points = 0

                # ARM BARRIER (SCAN-006): arm the Time Tagger measurements BEFORE starting the Pulse
                # Streamer, else a variable number of the first 'detect' edges are lost.
                try:
                    self._tagger.sync()
                except Exception:
                    pass
                if self._cbm_arm_delay and float(self._cbm_arm_delay) > 0:
                    time.sleep(float(self._cbm_arm_delay))

                # Pulse Streamer: play the point block n times (advances the SMIQ + gates the TT).
                # IMMEDIATE start so it runs as soon as streamed (the TT is already armed above);
                # final=ZERO so the detect line drops LOW after the last point -> the last count
                # window's closing edge occurs and CountBetweenMarkers completes.
                self._pulser.setTrigger(start=ps.TriggerStart.IMMEDIATE)
                self._pulser.stream(self._build_point_sequence(), n, ps.OutputState.ZERO())
                self._frame_start_time = time.perf_counter()
            except Exception:
                self._teardown()
                self.module_state.unlock()
                raise

    def stop_buffered_acquisition(self):
        # Must NOT raise if nothing is running (interface contract).
        with self._lock:
            if self.module_state() == 'locked':
                self._teardown()
                self.module_state.unlock()

    def _teardown(self):
        # Return the Pulse Streamer to idle. If keep_laser_on (and the laser is enabled), HOLD the
        # laser channel HIGH so the PL source keeps steady illumination between sweep lines and after
        # the scan (no blink -> no re-bleaching transient); otherwise all outputs LOW. mw switch and
        # the advance line always go LOW here. on_deactivate forces all-LOW for a self-owned Streamer.
        try:
            if self._pulser is not None:
                if self._keep_laser_on and self._enable_laser:
                    self._pulser.constant(ps.OutputState([int(self._ps_channels['laser'])], 0, 0))
                else:
                    self._pulser.constant(ps.OutputState.ZERO())
        except Exception:
            pass
        # Defensively stop the Time Tagger measurements before dropping references (guarded).
        for _meas in (self._cbm, self._detect_monitor):
            try:
                if _meas is not None:
                    _meas.stop()
            except Exception:
                pass
        self._cbm = None
        self._combiner = None
        self._detect_monitor = None

    def laser_off(self):
        """ Turn the laser OFF (Pulse Streamer all outputs LOW). Use when done with a continuous-laser
        (keep_laser_on) ODMR session to stop illumination. Refuses during a running sweep. """
        with self._lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Refusing to change the laser during a running ODMR sweep.')
            if self._pulser is not None:
                self._pulser.constant(ps.OutputState.ZERO())
        self.log.info('Laser OFF: Pulse Streamer outputs all zero.')

    def get_tagger(self):
        """ Return the underlying TimeTagger object so another module (e.g. a live photon counter)
        can run additional measurements on the SAME device (multiple measurements per connection). """
        return self._tagger

    # ---------------------------------------------------------------- readout
    def _windows_to_points(self, counts):
        """ Convert the per-window photon counts (length frame_size * windows_per_point) to one value
        per frequency point, using each window's actual integration time.
        sum  -> count rate (c/s) of the single window.
        diff -> rate(A, mw ON) - rate(B, mw OFF): the common PL baseline (bleaching) cancels, leaving
                the mw-induced change (a dip at resonance on a ~0 baseline). """
        counts = np.asarray(counts, dtype=np.float64)
        if self._windows_per_point == 2:
            ab = counts.reshape(-1, 2)
            ta = self._win_a_time if self._win_a_time else 1.0
            tb = self._win_b_time if self._win_b_time else ta
            return ab[:, 0] / ta - ab[:, 1] / tb
        win_time = self._win_a_time if self._win_a_time else \
            (float(self._accumulate_fraction) / self._sample_rate)
        return counts / win_time

    def _available_points_total(self):
        """ Points whose count window has CLOSED, read from the Time Tagger itself.
        CountBetweenMarkers.getBinWidths() returns each window's accumulation time: a closed window
        has width > 0, a not-yet-closed window has width 0 -- independent of photon count, so it works
        with the laser off (SCAN-002). windows_per_point windows per point (1 sum, 2 diff): a point is
        complete only when ALL its windows have closed. """
        if self._cbm is None:
            return 0
        try:
            closed = int(np.count_nonzero(np.asarray(self._cbm.getBinWidths())))
        except Exception:
            closed = 0
        return max(0, min(closed // self._windows_per_point, self._frame_size))

    def _log_frame_timeout(self, timeout):
        try:
            detect_rate = float(self._detect_monitor.getData()[0])
        except Exception:
            detect_rate = -1.0
        try:
            closed = int(np.count_nonzero(np.asarray(self._cbm.getBinWidths())))
        except Exception:
            closed = -1
        self.log.error(
            f'ODMR sweep line timed out after {timeout:.1f} s ({self._frame_size} points @ '
            f'{self._sample_rate} Hz). DIAGNOSTICS: detect-edge rate = {detect_rate:.0f}/s '
            f'(expect ~{self._sample_rate:.0f}); closed windows = {closed} of {self._frame_size}. '
            f'detect~0 -> markers not reaching the Time Tagger (check ch1->TT ch5); '
            f'closed~N-1 -> last window not closing (tail/final-state). Returning zeros for the rest.')

    def get_buffered_samples(self, number_of_samples=None):
        with self._lock:
            if self.module_state() != 'locked':
                raise RuntimeError('Unable to read samples. Acquisition is not running.')
            remaining_in_frame = self._frame_size - self._consumed_points
            if number_of_samples is not None and int(number_of_samples) > remaining_in_frame:
                raise ValueError(f'Requested {number_of_samples} samples but only '
                                 f'{remaining_in_frame} pending in this frame')
            target = self._available_points_total() - self._consumed_points \
                if number_of_samples is None else int(number_of_samples)
            if target < 1:
                return {ch: np.empty(0, dtype=np.float64) for ch in self._active_channels}

            timeout = float(self._frame_timeout) if self._frame_timeout else \
                (self._frame_size / max(self._sample_rate, 1e-9)) * 2.0 + 5.0
            deadline = time.perf_counter() + timeout
            while (self._available_points_total() - self._consumed_points) < target:
                if time.perf_counter() > deadline:
                    self._log_frame_timeout(timeout)
                    remaining = self._frame_size - self._consumed_points
                    self._consumed_points = self._frame_size
                    return {ch: np.zeros(remaining, dtype=np.float64) for ch in self._active_channels}
                time.sleep(min(max(1.0 / self._sample_rate, 1e-3), 0.05))

            points = self._windows_to_points(np.asarray(self._cbm.getData()))  # length frame_size
            block = points[self._consumed_points:self._consumed_points + target]
            self._consumed_points += target
            return {ch: np.asarray(block, dtype=np.float64) for ch in self._active_channels}

    def acquire_frame(self, frame_size=None):
        """ Acquire one full sweep line (blocking): configure, run, and return the per-point counts.

        OdmrLogic calls this once per sweep with no argument (it has already set the frame size via
        set_frame_size). Mirrors the finite_sampling_input_dummy contract.
        """
        with self._lock:
            restore = None
            if frame_size is not None:
                restore = self._frame_size
                samples = int(round(frame_size))
                if not self._constraints.frame_size_in_range(samples)[0]:
                    raise ValueError(f'frame size {samples} out of bounds '
                                     f'{self._constraints.frame_size_limits}')
                self._frame_size = samples
        try:
            self.start_buffered_acquisition()
            data = self.get_buffered_samples(self.frame_size)
            self.stop_buffered_acquisition()
            return data
        finally:
            if restore is not None:
                with self._lock:
                    self._frame_size = restore
