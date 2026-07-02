# -*- coding: utf-8 -*-

"""
Qudi hardware module (confocal_odmr): FiniteSamplingIO that produces one confocal scan frame by
coordinating three devices, all driven from the Pulse Streamer master clock:

  * NI AO  -- piezo x/y/z voltages played as a JUMP_LIST, EXTERNALLY sample-clocked on PFI0
  * Time Tagger -- per-pixel gated photon counts (CountBetweenMarkers on the 'detect' marker)
  * Pulse Streamer -- per-pixel sequence: mw-switch gate (ch4), detect count-gate (ch1),
                      pixel-advance clock (ch5 -> PFI0), and (optionally) laser gate (ch0)

It implements FiniteSamplingIOInterface so the stock ni_scanning_probe_interfuse + scanning
logic + Scanner GUI can be reused unchanged (see Qudi_AI/setups/confocal_odmr/
confocal_scanner_design.md). Written from scratch from the operator's spec (2026-06-26).

Per-pixel timing (pixel duration T = 1/sample_rate; accumulate fraction default 0.9):
    ch4 mw     : HIGH [0, T/2)            LOW [T/2, T)
    ch1 detect : HIGH [skip, T/2)  and  HIGH [T/2+skip, T)     where skip = (1-accumulate)/2 * T
    ch5 next   : short rising pulse at the end of the pixel  -> advances NI AO
    ch0 laser  : HIGH for whole scan IFF enable_laser (human-approved); else untouched/off
The Time Tagger returns 2 windows/pixel: A (mw-on half) and B (mw-off half).
    sum mode  : A + B   (normal confocal PL)
    diff mode : A - B   (mw response; requires SMIQ RF on continuously -- human approval)

SAFETY / STATUS: FIRST DRAFT. Counting and (with the piezo off) AO output are read-only-ish, but
this module drives the piezo (motion) and can gate the laser. Validate with the piezo OFF and
laser OFF first (dark-count image). Lines marked `# VALIDATE` need a hardware dry-run to confirm
(esp. NI external-clock alignment / pixel_next off-by-one, getData timing, normalization).
Measurement correctness needs human/expert validation. Codex review advised.

This file is part of qudi. Licensed under LGPL v3.
"""

import time
import numpy as np
from typing import Dict, List, Optional, Sequence as TSequence, Union

import nidaqmx
from nidaqmx.constants import AcquisitionType, Edge
import pulsestreamer as ps
import TimeTagger as tt

from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
from qudi.util.constraints import ScalarConstraint
from qudi.interface.finite_sampling_io_interface import (FiniteSamplingIOInterface,
                                                         FiniteSamplingIOConstraints)
from qudi.util.enums import SamplingOutputMode


class ConfocalScanIO(FiniteSamplingIOInterface):
    """
    Example config:

    confocal_scan_io:
        module.Class: 'interfuse.confocal_scan_io.ConfocalScanIO'
        options:
            ni_device: 'Dev1'
            ao_channels: ['ao0', 'ao1', 'ao2']  # NI AO terminals = this module's output channel
                                                # names. The ni_scanning_probe_interfuse maps the
                                                # scan axes x/y/z -> these via its ni_channel_mapping.
            ao_voltage_limits: [0.0, 10.0]      # piezo input range (human-confirmed)
            ni_sample_clock_terminal: '/Dev1/PFI0'   # external clock from pulse_streamer ch5
            pulsestreamer_ip: '169.254.8.2'
            ps_channels:                 # Pulse Streamer digital channels (connections.yaml)
                mw: 4
                detect: 1
                pixel_next: 5
                laser: 0
            timetagger_serial: ''        # optional; '' -> the only Time Tagger
            apd_channels: [1, 2]         # Time Tagger click channels (APD1, APD2)
            detect_tt_channel: 5         # Time Tagger channel receiving the 'detect' marker
            input_channel_name: 'fluorescence'
            accumulate_fraction: 0.9     # fraction of the pixel actually counted (tunable)
            scan_mode: 'sum'             # 'sum' (PL) or 'diff' (mw response; needs RF on)
            pixel_next_pulse_ns: 100     # width of the AO-advance pulse
            enable_laser: False          # True drives ch0 laser HIGH during scan -- HUMAN APPROVAL
            default_sample_rate: 200.0   # Hz -> 5 ms pixel
    """
    _ni_device = ConfigOption('ni_device', default='Dev1', missing='warn')
    _ao_channels = ConfigOption('ao_channels', missing='error')
    _ao_voltage_limits = ConfigOption('ao_voltage_limits', default=(0.0, 10.0))
    _ni_clk_terminal = ConfigOption('ni_sample_clock_terminal', default='/Dev1/PFI0')
    _ps_ip = ConfigOption('pulsestreamer_ip', missing='error')
    _ps_channels = ConfigOption('ps_channels', missing='error')
    _tt_serial = ConfigOption('timetagger_serial', default='')
    _apd_channels = ConfigOption('apd_channels', missing='error')
    _detect_tt_channel = ConfigOption('detect_tt_channel', default=5)
    _input_channel_name = ConfigOption('input_channel_name', default='fluorescence')
    _accumulate_fraction = ConfigOption('accumulate_fraction', default=0.9)
    _scan_mode = ConfigOption('scan_mode', default='sum')
    _pixel_next_pulse_ns = ConfigOption('pixel_next_pulse_ns', default=100)
    # Per-pixel settle gap (seconds) at the START of each pixel block: the piezo is advanced to this
    # pixel's position by the previous block's pixel_next pulse, and this gap lets it ARRIVE before
    # the count windows open, so each pixel's PL is taken at the settled position instead of
    # mid-transit. Without it, counting during the move biases each pixel toward the previous
    # position (forward image ~1 px low, backward ~1 px high). Costs count time (shrinks the count
    # window), so lower the sample_rate if you need it back. HARDWARE-SPECIFIC -- tune to the piezo;
    # 0 keeps the original behaviour. (SCAN-007)
    _pixel_settle_time = ConfigOption('pixel_settle_time', default=0.0)
    # Advance the NI AO at the START of each pixel (trigger -> settle -> count) instead of the end.
    # The NI AO emits its first sample on the FIRST pixel_next edge, so with the advance at the pixel
    # END each pixel is counted at the PREVIOUS position p_{i-1} -> a fixed 1-px (2-px fwd/bwd)
    # registration offset (confirmed rate-independent + pixel-fixed, SCAN-007). Advancing at the start
    # counts pixel i at p_i. True is the fix; False = legacy (advance at end). Verify with the
    # forward/backward peak separation -> 0.
    _pixel_next_at_start = ConfigOption('pixel_next_at_start', default=True)
    _enable_laser = ConfigOption('enable_laser', default=False)
    # idle_laser_on: hold the laser ON while the module is idle (not scanning) — for cursor /
    # positioning tests where you watch the back-reflection. HUMAN-APPROVED continuous laser
    # output; default False. (enable_laser above controls the laser DURING a scan sequence.)
    _idle_laser_on = ConfigOption('idle_laser_on', default=False)
    _default_sample_rate = ConfigOption('default_sample_rate', default=200.0)
    # safety net: max seconds to wait for a Time Tagger frame before aborting (so a missing detect
    # marker can't block the scan forever). None -> auto (expected frame time x2 + 5 s).
    _frame_timeout = ConfigOption('frame_timeout', default=None)
    # Post-ramp piezo settle (seconds): a SHORT dwell at the start of every buffered frame, i.e. AFTER
    # the reused ni_scanning_probe_interfuse has ramped the AO to the first scan position and BEFORE the
    # first pixel is clocked. The interfuse starts the scan the instant the AO *setpoint* reaches
    # target, with no wait for the physical piezo. The velocity ramp (interfuse maximum_move_velocity)
    # handles the DISTANCE-DEPENDENT part of the move; what remains at ramp-end is a small, roughly
    # CONSTANT following-error residual (~ velocity * piezo_time_constant, independent of move size),
    # which this fixed dwell lets decay. So -- unlike a settle-only scheme -- a small fixed value works
    # regardless of how far the stage moved. The AO holds the target during the dwell (nicard_ao uses
    # keep_value: True). HARDWARE-SPECIFIC: tune to the piezo; 0 disables. (SCAN-005)
    _settle_time = ConfigOption('settle_time', default=0.05)
    # Arm the Time Tagger measurements before starting the Pulse Streamer clock. A freshly created
    # CountBetweenMarkers needs ~tens of ms to start listening; without this, the first detect edges
    # are lost by a variable amount -> count<->position registration slip (SCAN-006). Applied on top
    # of tagger.sync(); >=0.05 s was proven sufficient at 1 kHz (probe_scan_registration.py).
    _cbm_arm_delay = ConfigOption('cbm_arm_delay', default=0.05)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = Mutex()
        self._tagger = None
        self._pulser = None
        self._constraints = None
        # active scan/frame state
        self._sample_rate = 0.0
        self._frame_size = 0
        self._output_mode = SamplingOutputMode.JUMP_LIST
        self._active_out = []
        self._active_in = []
        self._frame_voltages = None         # dict ao-axis -> np.ndarray (N,)
        self._ao_task = None
        self._cbm = None                    # TimeTagger.CountBetweenMarkers
        self._combiner = None
        self._detect_monitor = None         # TimeTagger.Countrate on the detect channel (diagnostic)
        self._consumed_pixels = 0
        self._frame_pixels = None
        self._laser_is_on = False   # tracks the intended laser state (idle config / laser_on())
        self._frame_start_time = None
        # Samples snapshotted at teardown so get_buffered_samples can still drain them AFTER the
        # frame has stopped (stock ni_x_series_finite_sampling_io contract; see SCAN-003).
        self._unread = None
        # per-window integration time (s), set when the pixel sequence is built (depends on the
        # per-pixel settle gap); used for the c/s normalization in _windows_to_pixels (SCAN-007).
        self._win_time = None      # window A duration (s) -- diff mode / fallback
        self._sum_time = None      # actual total count-window duration win_a+win_b (s) -- sum mode

    # ---------------------------------------------------------------- lifecycle
    def on_activate(self):
        # Validate timing-critical config up front (before touching hardware) so a typo fails fast
        # with a clear message instead of producing negative Pulse Streamer segment durations or a
        # late error mid-scan (Codex review #4).
        if not (0.0 < float(self._accumulate_fraction) < 1.0):
            raise ValueError(f'accumulate_fraction must be in (0, 1), got {self._accumulate_fraction}')
        if int(self._pixel_next_pulse_ns) <= 0:
            raise ValueError(f'pixel_next_pulse_ns must be positive, got {self._pixel_next_pulse_ns}')
        if str(self._scan_mode).lower() not in ('sum', 'diff'):
            raise ValueError(f"scan_mode must be 'sum' or 'diff', got {self._scan_mode}")
        # Timing gaps/delays must be non-negative: a negative pixel_settle_time would feed a negative
        # segment into _build_pixel_sequence, which _seg() would silently drop and deform the pulse
        # pattern instead of failing loudly (Codex review). settle_time/cbm_arm_delay < 0 are benign
        # (guarded at use) but rejected here too for a clear failure.
        for _name in ('_pixel_settle_time', '_settle_time', '_cbm_arm_delay'):
            _val = getattr(self, _name)
            if _val is not None and float(_val) < 0:
                raise ValueError(f'{_name[1:]} must be >= 0, got {_val}')
        self._pulser = ps.PulseStreamer(self._ps_ip)
        self._tagger = tt.createTimeTagger(self._tt_serial) if self._tt_serial else tt.createTimeTagger()
        # Set a defined output state at activation.
        if self._idle_laser_on:
            self._pulser.constant(ps.OutputState([int(self._ps_channels['laser'])], 0, 0))
            self._laser_is_on = True
            self.log.warning('LASER ON at activation (idle_laser_on=True): human-approved continuous '
                             'laser output for a positioning test; Pulse Streamer laser channel HIGH.')
        else:
            self._pulser.constant(ps.OutputState.ZERO())  # all outputs LOW (laser/mw OFF)
            self._laser_is_on = False

        lo, hi = float(self._ao_voltage_limits[0]), float(self._ao_voltage_limits[1])
        out_units = {ax: 'V' for ax in self._ao_channels}
        out_limits = {ax: (lo, hi) for ax in self._ao_channels}
        in_units = {self._input_channel_name: 'c/s'}
        self._constraints = FiniteSamplingIOConstraints(
            supported_output_modes=(SamplingOutputMode.JUMP_LIST,),
            input_channel_units=in_units,
            output_channel_units=out_units,
            input_channel_limits={self._input_channel_name: (-np.inf, np.inf)},
            output_channel_limits=out_limits,
            frame_size_limits=(1, int(1e7)),
            sample_rate_limits=(0.1, 1e4),  # VALIDATE vs Pulse Streamer / piezo bandwidth
        )
        self._sample_rate = float(self._default_sample_rate)
        self._active_out = list(self._ao_channels)
        self._active_in = [self._input_channel_name]

    def on_deactivate(self):
        try:
            self.stop_buffered_frame()
        except Exception:
            pass
        if self._pulser is not None:
            try:
                self._pulser.constant(ps.OutputState.ZERO())  # leave all outputs LOW (safe)
            except Exception:
                pass
        if self._tagger is not None:
            tt.freeTimeTagger(self._tagger)
            self._tagger = None
        self._pulser = None

    # ---------------------------------------------------------------- properties
    @property
    def constraints(self):
        return self._constraints

    @property
    def active_channels(self):
        return list(self._active_in), list(self._active_out)

    @property
    def sample_rate(self):
        return self._sample_rate

    @property
    def frame_size(self):
        return self._frame_size

    @property
    def output_mode(self):
        return self._output_mode

    @property
    def is_running(self):
        """ True while a buffered frame is being acquired (module_state locked).
        Not part of the FiniteSamplingIOInterface ABC, but the ni_scanning_probe_interfuse
        relies on it (e.g. in on_deactivate), so the stock NI module and this one both provide it.
        """
        return self.module_state() == 'locked'

    @property
    def samples_in_buffer(self):
        # After a frame has stopped, report whatever was snapshotted as still-unread so the
        # interfuse can drain it; never report a stale live count when not locked (see SCAN-003).
        if self.module_state() != 'locked':
            if self._unread is not None:
                return int(len(self._unread.get(self._input_channel_name, [])))
            return 0
        if self._cbm is None:
            return 0
        return max(0, self._available_pixels_total() - self._consumed_pixels)

    # ---------------------------------------------------------------- configuration
    def set_sample_rate(self, rate):
        lo, hi = self._constraints.sample_rate_limits
        if not (lo <= float(rate) <= hi):
            raise ValueError(f'Sample rate {rate} out of bounds {(lo, hi)}')
        self._sample_rate = float(rate)

    def set_active_channels(self, input_channels, output_channels):
        self._active_in = list(input_channels)
        self._active_out = list(output_channels)

    def set_pixel_settle_time(self, seconds):
        """ Runtime setter for the per-pixel settle gap (s), so it can be swept from the qudi console
        without relaunching (SCAN-007 tuning). Takes effect on the NEXT scan; refuses during one. """
        if float(seconds) < 0:
            raise ValueError(f'pixel_settle_time must be >= 0, got {seconds}')
        with self._lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Cannot change pixel_settle_time during a running scan frame.')
            self._pixel_settle_time = float(seconds)
        self.log.info(f'pixel_settle_time set to {self._pixel_settle_time} s (effective next scan).')

    def set_output_mode(self, mode):
        if SamplingOutputMode(mode) != SamplingOutputMode.JUMP_LIST:
            raise ValueError('ConfocalScanIO only supports JUMP_LIST output mode')
        self._output_mode = SamplingOutputMode.JUMP_LIST

    def set_frame_data(self, data):
        if data is None:
            self._frame_voltages = None
            self._frame_size = 0
            return
        sizes = {len(v) for v in data.values()}
        if len(sizes) != 1:
            raise ValueError('All output channel arrays must have equal length')
        self._frame_voltages = {ch: np.asarray(v, dtype=np.float64) for ch, v in data.items()}
        self._frame_size = sizes.pop()

    # ---------------------------------------------------------------- Pulse Streamer sequence
    def _build_pixel_sequence(self):
        """ One pixel block (duration T = 1/sample_rate), streamed n_runs = frame_size times.

        Structure: [settle gap][count region: window A (mw ON) | window B (mw OFF)][pixel_next].
        The piezo is advanced to THIS pixel's position by the PREVIOUS block's pixel_next pulse (at
        the block boundary); the settle gap at the START of the block lets the stage arrive there
        before the count windows open, so each pixel's PL is taken at the settled position rather
        than mid-transit. Without it, counting during the move biases each pixel toward the previous
        position (forward image ~1 px low, backward ~1 px high). pixel_settle_time = 0 keeps the
        original behaviour. (SCAN-007)
        """
        T_ns = int(round(1e9 / self._sample_rate))
        pw = int(min(self._pixel_next_pulse_ns, max(1, T_ns // 100)))
        settle = int(round(float(self._pixel_settle_time) * 1e9))
        L = T_ns - pw - settle                       # length of the counting region
        half = L // 2
        skip = int(round((1.0 - float(self._accumulate_fraction)) / 2.0 * L))
        tail = pw
        win_a = half - skip                          # window A (mw ON) duration
        win_b = L - half - skip - tail               # window B (mw OFF) duration
        if win_a <= 0 or win_b <= 0:
            raise RuntimeError(
                f'Invalid pixel timing: pixel_settle_time={self._pixel_settle_time} s leaves count '
                f'windows A={win_a} ns, B={win_b} ns at {self._sample_rate} Hz (pixel {T_ns} ns). '
                f'Lower pixel_settle_time or the sample rate.')
        ch = self._ps_channels
        # per-window (A) integration time (s); window B is shorter by 'tail' (the SCAN-001 tail low).
        # _win_time approximates window A and is used for diff mode (which is known-open, needs RF).
        # _sum_time is the ACTUAL total count-window duration (win_a + win_b) for exact sum-mode c/s
        # normalization -- matters if pixel_next_pulse_ns is increased (Codex review).
        self._win_time = (win_a / 1e9)
        self._sum_time = (win_a + win_b) / 1e9

        def _seg(pairs):
            return [(int(d), int(v)) for (d, v) in pairs if int(d) > 0]  # drop zero-length segments

        seq = self._pulser.createSequence()
        if self._pixel_next_at_start:
            # [pixel_next advance][settle][window A (mw ON)][window B (mw OFF)]. Advance the AO to THIS
            # pixel's position at the very start; the NI AO emits its first sample on the first
            # pixel_next edge, so advancing first counts pixel i at p_i (fixes the 1-px offset). Then
            # settle, then count. (SCAN-007)
            off = pw + settle                            # counting starts after advance + settle
            seq.setDigital(ch['pixel_next'], _seg([(pw, 1), (T_ns - pw, 0)]))
            seq.setDigital(ch['mw'], _seg([(off, 0), (half, 1), (L - half, 0)]))
            seq.setDigital(ch['detect'], _seg([(off + skip, 0), (win_a, 1),
                                               (skip, 0), (win_b, 1), (tail, 0)]))
        else:
            # legacy: [settle][window A][window B][pixel_next]  (advance at the END; 1-px offset)
            seq.setDigital(ch['pixel_next'], _seg([(T_ns - pw, 0), (pw, 1)]))
            seq.setDigital(ch['mw'], _seg([(settle, 0), (half, 1), (L - half + pw, 0)]))
            seq.setDigital(ch['detect'], _seg([(settle + skip, 0), (win_a, 1),
                                               (skip, 0), (win_b, 1), (tail, 0), (pw, 0)]))
        # laser: held HIGH for the whole pixel if the laser is meant to be ON during the scan
        # (enable_laser) or was already on for positioning (idle_laser_on). Otherwise left LOW.
        if self._enable_laser or self._laser_is_on:
            seq.setDigital(ch['laser'], [(T_ns, 1)])
        return seq

    # ---------------------------------------------------------------- frame control
    def start_buffered_frame(self):
        with self._lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Frame already running')
            if self._frame_voltages is None or self._frame_size < 1:
                raise RuntimeError('No frame data set')
            # Every active output channel must have frame voltages, else the np.vstack below raises a
            # late, opaque KeyError. Fail early with a clear message (Codex review #4).
            missing = [ax for ax in self._active_out if ax not in self._frame_voltages]
            if missing:
                raise RuntimeError(f'Frame data missing voltages for active output channel(s) '
                                   f'{missing}; have {list(self._frame_voltages)}')
            self.module_state.lock()
            try:
                # Post-ramp piezo settle: the interfuse has just finished ramping the AO to the first
                # scan position and starts us with NO wait for the stage to catch up. Dwell a short,
                # fixed time so the residual following-error decays before the first pixel is clocked.
                # The AO holds the target during this (nicard_ao keep_value: True), and we haven't
                # created our AO task yet, so nothing perturbs the output. The velocity ramp does the
                # bulk (distance-dependent) move; this only covers the small constant residual. SCAN-005.
                if self._settle_time and float(self._settle_time) > 0:
                    time.sleep(float(self._settle_time))
                n = self._frame_size
                # 1) NI AO buffered task, EXTERNALLY clocked on PFI0 (from pulse_streamer ch5)
                self._ao_task = nidaqmx.Task()
                for ch in self._active_out:                 # ch is the NI AO terminal (e.g. 'ao0')
                    term = f"{self._ni_device}/{ch}"
                    lo, hi = self._ao_voltage_limits
                    self._ao_task.ao_channels.add_ao_voltage_chan(term, min_val=lo, max_val=hi)
                self._ao_task.timing.cfg_samp_clk_timing(
                    rate=self._sample_rate,                      # hint; real timing is external
                    source=self._ni_clk_terminal,               # '/Dev1/PFI0'
                    active_edge=Edge.RISING,
                    sample_mode=AcquisitionType.FINITE,
                    samps_per_chan=n)
                # write JUMP_LIST in active-channel order  # VALIDATE: pixel_next vs sample alignment
                write_arr = np.vstack([self._frame_voltages[ax] for ax in self._active_out])
                self._ao_task.write(write_arr, auto_start=False)
                self._ao_task.start()  # armed; waits for external clock edges

                # 2) Time Tagger: combine APDs, count between detect rising/falling -> 2 windows/pixel
                self._combiner = tt.Combiner(self._tagger, channels=list(self._apd_channels))
                self._cbm = tt.CountBetweenMarkers(
                    self._tagger,
                    click_channel=self._combiner.getChannel(),
                    begin_channel=int(self._detect_tt_channel),
                    end_channel=-int(self._detect_tt_channel),  # falling edge of same channel
                    n_values=2 * n)
                # diagnostic: count detect edges actually arriving at the Time Tagger during the frame
                self._detect_monitor = tt.Countrate(self._tagger, [int(self._detect_tt_channel)])
                self._consumed_pixels = 0
                self._frame_pixels = None
                self._unread = None   # clear any leftover snapshot from a previous frame

                # ARM BARRIER (SCAN-006): a freshly created CountBetweenMarkers takes time (~tens of
                # ms) to actually start listening. If the Pulse Streamer starts before it is armed, a
                # VARIABLE number of the first 'detect' edges are lost, so the count<->position
                # registration slips by a random integer pixel each frame -- invisible in a single 2D
                # frame (one global offset), but visible as run-to-run drift in repeated 1D line scans.
                # tagger.sync() is the Time Tagger's barrier (blocks until the measurements are
                # initialized); the short fixed cbm_arm_delay is a proven belt-and-suspenders margin
                # (probe_scan_registration.py: >=0.05 s -> 200/200 windows every frame). Guarded so a
                # missing sync() can't break start-up -- the delay alone already fixes it.
                try:
                    self._tagger.sync()
                except Exception:
                    pass
                if self._cbm_arm_delay and float(self._cbm_arm_delay) > 0:
                    time.sleep(float(self._cbm_arm_delay))

                # 3) Pulse Streamer: play the pixel block n times (drives AO + gates TT).
                # Software-trigger + explicit startNow() so the clock starts AFTER NI AO and the
                # Time Tagger are armed. Without an explicit start, stream() can leave the sequence
                # idle -> no detect markers -> the Time Tagger never completes (the observed hang).
                # Immediate-start trigger: the sequence runs as soon as it is streamed (NI AO and
                # the Time Tagger are already armed above, so they catch the first clock/markers).
                # final=ZERO so the detect line drops LOW after the last pixel -> that final detect
                # falling edge closes the last count window. Without it, CountBetweenMarkers waits
                # forever for the (2*N)-th window's closing marker and the frame times out.
                self._pulser.setTrigger(start=ps.TriggerStart.IMMEDIATE)
                self._pulser.stream(self._build_pixel_sequence(), n, ps.OutputState.ZERO())
                self._frame_start_time = time.perf_counter()
            except Exception:
                self._teardown()
                self.module_state.unlock()
                raise

    def stop_buffered_frame(self):
        with self._lock:
            if self.module_state() == 'locked':
                self._teardown()
                self.module_state.unlock()

    def _teardown(self):
        # Snapshot only the pixels actually ACQUIRED (both count windows CLOSED) and not yet
        # consumed, BEFORE releasing the CountBetweenMarkers measurement, so get_buffered_samples can
        # hand them back after the frame stops. The stock ni_x_series_finite_sampling_io contract is
        # "return samples already in the buffer", NOT all remaining frame positions: unclosed tail
        # windows read as zeros via getData() and must not be passed off as real data on a manual
        # stop / abort / genuine tail shortfall (Codex review #2). getBinWidths() gives the closed-
        # window count independent of photon count (same basis as SCAN-002).
        try:
            if self._cbm is not None and self._frame_size:
                closed_windows = int(np.count_nonzero(np.asarray(self._cbm.getBinWidths())))
                available = max(0, min(closed_windows // 2, self._frame_size))
                pixels = self._windows_to_pixels(np.asarray(self._cbm.getData()))
                remaining = np.asarray(pixels[self._consumed_pixels:available], dtype=np.float64)
                self._unread = {self._input_channel_name: remaining}
        except Exception:
            self._unread = {self._input_channel_name: np.array([], dtype=np.float64)}
        try:
            if self._pulser is not None:
                # Return to the intended idle state: keep the laser ON if it was on (so it does not
                # blink off between scan frames), else all outputs LOW. on_deactivate forces zero
                # for a safe shutdown.
                if self._laser_is_on:
                    self._pulser.constant(ps.OutputState([int(self._ps_channels['laser'])], 0, 0))
                else:
                    self._pulser.constant(ps.OutputState.ZERO())
        except Exception:
            pass
        # NI AO cleanup must never escape _teardown -- otherwise stop_buffered_frame()/
        # start_buffered_frame() could leave module_state locked (Codex review #3). Guard stop() and
        # close() separately and always drop the task reference.
        if self._ao_task is not None:
            try:
                self._ao_task.stop()
            except Exception:
                pass
            try:
                self._ao_task.close()
            except Exception:
                pass
            self._ao_task = None
        # Defensively stop the Time Tagger measurements before dropping the references, mirroring the
        # upstream Time Tagger pattern (Codex review #5). Guarded so a missing method can't break
        # teardown. (Combiner is a virtual channel, not a measurement -- just drop it.)
        for _meas in (self._cbm, self._detect_monitor):
            try:
                if _meas is not None:
                    _meas.stop()
            except Exception:
                pass
        self._cbm = None
        self._combiner = None
        self._detect_monitor = None

    # ---------------------------------------------------------------- manual laser control
    def laser_on(self):
        """ Turn the laser ON continuously: Pulse Streamer 'laser' channel HIGH, all others LOW.

        HUMAN-APPROVED LASER OUTPUT. Intended for static positioning tests (drag the cursor in the
        Scanner GUI -> the piezo moves -> watch the back-reflection). Single owner of the Pulse
        Streamer connection, so there is no device contention with the scan path. Refuses to run
        while a scan frame is active. Call laser_off() to switch it back off.
        """
        with self._lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Refusing to toggle the laser during a running scan frame.')
            self._pulser.constant(ps.OutputState([int(self._ps_channels['laser'])], 0, 0))
            self._laser_is_on = True
            self.log.warning('LASER ON (continuous): Pulse Streamer laser channel held HIGH '
                             '(human-approved laser output). Call laser_off() when done.')

    def laser_off(self):
        """ Switch the laser OFF: Pulse Streamer outputs all zero. """
        with self._lock:
            self._pulser.constant(ps.OutputState.ZERO())
            self._laser_is_on = False
            self.log.info('Laser OFF: Pulse Streamer outputs all zero.')

    def get_tagger(self):
        """ Return the underlying TimeTagger object so another module (e.g. a live photon counter)
        can run additional measurements on the SAME device. The Time Tagger supports multiple
        concurrent measurements on one connection, whereas a second createTimeTagger() for the same
        device would fail. """
        return self._tagger

    # ---------------------------------------------------------------- readout
    def _windows_to_pixels(self, counts_2n):
        # per-window integration time: set by _build_pixel_sequence (accounts for the settle gap);
        # fall back to the settle-free estimate if a frame hasn't been built yet.
        win_time = self._win_time if self._win_time else \
            (float(self._accumulate_fraction) / 2.0) * (1.0 / self._sample_rate)
        sum_time = self._sum_time if self._sum_time else (2.0 * win_time)  # actual A+B window time
        ab = np.asarray(counts_2n, dtype=np.float64).reshape(-1, 2)  # (pixels, [A, B])
        a, b = ab[:, 0], ab[:, 1]
        if str(self._scan_mode).lower() == 'diff':
            return (a - b) / win_time                 # VALIDATE: diff normalization (known-open; needs RF)
        return (a + b) / sum_time                     # sum -> mean count rate in c/s (exact A+B window)

    def _available_pixels_total(self):
        """ Pixels whose BOTH count windows have actually closed, read from the Time Tagger itself.
        CountBetweenMarkers.getBinWidths() returns the accumulation time of each window: a closed
        window has width > 0, a not-yet-closed window has width 0 -- independent of photon counts,
        so it works with the laser off (unlike counting nonzero photon counts). 2 windows per pixel.
        The closed-window count is the authority. """
        if self._cbm is None:
            return 0
        try:
            closed_windows = int(np.count_nonzero(np.asarray(self._cbm.getBinWidths())))
        except Exception:
            closed_windows = 0
        completed = closed_windows // 2
        # AO-done snap-to-full is only valid when pixel_next is at the pixel END: there the AO's last
        # sample is clocked at the END of the last pixel, so is_task_done() implies the last pixel's
        # count windows have closed. With pixel_next_at_start=True (SCAN-007) the last AO sample is
        # clocked at the START of the last pixel, BEFORE its detect windows close -- snapping there
        # would return zero/partial data for the final pixel. So only snap in the legacy ordering;
        # otherwise trust the closed-window count (the frame timeout is the backstop). (Codex review)
        if not self._pixel_next_at_start:
            try:
                if self._ao_task is not None and self._ao_task.is_task_done():
                    return self._frame_size
            except Exception:
                pass
        return max(0, min(completed, self._frame_size))

    def _log_scan_timeout(self, timeout):
        try:
            detect_rate = float(self._detect_monitor.getData()[0])
        except Exception:
            detect_rate = -1.0
        try:
            closed_windows = int(np.count_nonzero(np.asarray(self._cbm.getBinWidths())))
        except Exception:
            closed_windows = -1
        self.log.error(
            f'Scan frame timed out after {timeout:.1f} s ({self._frame_size} px @ '
            f'{self._sample_rate} Hz). DIAGNOSTICS: detect-edge rate during scan = '
            f'{detect_rate:.0f}/s (expect ~{2 * self._sample_rate:.0f}); CountBetweenMarkers '
            f'CLOSED windows (getBinWidths>0) = {closed_windows} of {2 * self._frame_size}. '
            f'detect~0 -> markers not reaching TT; closed~N -> 2nd window not closing; '
            f'closed~2N-1 -> last window. Returning blank for the rest of the frame.')

    def get_buffered_samples(self, number_of_samples=None):
        with self._lock:
            # Frame already stopped/finished: hand back any snapshotted unread samples and then
            # empty -- NEVER raise RuntimeError here. The stock ni_x_series_finite_sampling_io keeps
            # returning buffered/empty samples after a stop, and the ni_scanning_probe_interfuse
            # fetch loop polls once more after stopping (the optimizer runs many short scans, so it
            # hits this every time). Raising RuntimeError escaped the interfuse's `except ValueError`
            # into its generic handler -> a second stop_scan() -> unlock-while-idle FysomError
            # (SCAN-003). Match the stock contract: ValueError only when more is requested than
            # pending; otherwise drain.
            if self.module_state() != 'locked':
                buf = self._unread if self._unread is not None \
                    else {self._input_channel_name: np.array([], dtype=np.float64)}
                have = int(len(buf.get(self._input_channel_name, [])))
                if number_of_samples is None:
                    self._unread = {self._input_channel_name: np.array([], dtype=np.float64)}
                    return {k: np.asarray(v, dtype=np.float64) for k, v in buf.items()}
                n = int(number_of_samples)
                if n > have:
                    raise ValueError(f'Requested {n} samples but only {have} pending after stop')
                arr = np.asarray(buf[self._input_channel_name], dtype=np.float64)
                self._unread = {self._input_channel_name: arr[n:]}
                return {self._input_channel_name: arr[:n]}
            timeout = float(self._frame_timeout) if self._frame_timeout else \
                (self._frame_size / max(self._sample_rate, 1e-9)) * 2.0 + 5.0
            deadline = time.perf_counter() + timeout
            # Samples still belonging to this frame (not yet handed out). NEVER block for more than
            # this many: at end-of-frame the interfuse always asks for a full chunk (chunk_size=10)
            # even when only a couple of pixels remain (e.g. frame_size 512 or 32 -> 2 left). The
            # old code waited for the full chunk of NEW pixels that can never arrive once the frame
            # is complete, so it spun until the frame timeout (SCAN-004). Match the stock
            # ni_x_series_finite_sampling_io: raise ValueError when more is requested than the frame
            # has left -- the interfuse catches that (its `except ValueError`) and re-fetches without
            # a count, draining the remainder. Manual scans with a frame_size that is a multiple of
            # 10 never showed it; the optimizer's odd frame sizes do.
            pending_in_frame = self._frame_size - self._consumed_pixels
            if pending_in_frame <= 0:
                return {self._input_channel_name: np.array([], dtype=np.float64)}
            if number_of_samples is not None and int(number_of_samples) > pending_in_frame:
                raise ValueError(f'Requested {number_of_samples} samples but only '
                                 f'{pending_in_frame} pending in this frame')
            target = 1 if number_of_samples is None else int(number_of_samples)
            # Progressive: block only until enough NEW pixels are ready, so the interfuse fetches
            # and displays the image chunk-by-chunk (line-by-line) instead of all-at-once.
            while (self._available_pixels_total() - self._consumed_pixels) < target:
                if time.perf_counter() > deadline:
                    self._log_scan_timeout(timeout)
                    self._teardown()
                    remaining = self._frame_size - self._consumed_pixels
                    self._consumed_pixels = self._frame_size
                    # We return zeros for the ENTIRE rest of the frame here, so discard the
                    # teardown snapshot -- otherwise a later idle read would re-emit those same
                    # positions as real data on top of the zeros (Codex review #2).
                    self._unread = {self._input_channel_name: np.array([], dtype=np.float64)}
                    return {self._input_channel_name: np.zeros(remaining, dtype=np.float64)}
                time.sleep(min(max(1.0 / self._sample_rate, 1e-3), 0.05))
            avail = self._available_pixels_total() - self._consumed_pixels
            n = avail if number_of_samples is None else min(int(number_of_samples), avail)
            pixels = self._windows_to_pixels(np.asarray(self._cbm.getData()))  # length frame_size
            block = pixels[self._consumed_pixels:self._consumed_pixels + n]
            self._consumed_pixels += n
            return {self._input_channel_name: block}

    def get_frame(self, data):
        """ Convenience: configure output frame, run it, and return the input samples. """
        if data is not None:
            self.set_frame_data(data)
        self.start_buffered_frame()
        try:
            # Request the WHOLE frame (not a no-arg call, which after SCAN-002 returns only the
            # currently-available pixels and would let the finally-stop abort the frame early).
            # Matches the stock ni_x_series_finite_sampling_io.get_frame (Codex review #1).
            return self.get_buffered_samples(self.frame_size)
        finally:
            self.stop_buffered_frame()
