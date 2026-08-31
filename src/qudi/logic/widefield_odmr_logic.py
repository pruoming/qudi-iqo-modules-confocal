# -*- coding: utf-8 -*-

"""
Minimal widefield-ODMR logic — design ladder M3 (widefield_odmr setup, 2026-08-31).

Scope (deliberately minimal per the 2026-08-31 mandate/REQ-005): MW on/off differential
at a FIXED frequency — alternate gated/ungated camera frames, average N pairs, expose
the two mean images + mean contrast. NO frequency sweep, NO fitting, NO cursor (M4).

Frame-parity guard (design §3): finite bursts with trigger-count = frame-count; the
camera frame counter must be gapless and no extra frame may exist afterwards — any
violation aborts the burst with an error instead of silently swapping on/off roles.

SAFETY: enable_mw_output defaults to False — the MW source is CONFIGURED but never
switched on unless the config explicitly sets it (dummy-MW configs only, until the
setup's SAFE-xxx approvals exist). The gate channel drives the physical MW switch.
Ledger: Qudi_AI/setups/widefield_odmr/knowledge/fork_changes.yaml.
"""

import time
import numpy as np
from PySide6 import QtCore

from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from qudi.core.module import LogicBase
from qudi.util.mutex import RecursiveMutex


class WidefieldOdmrLogic(LogicBase):
    """ MW on/off differential imaging at fixed frequency (M3 minimal).

    Example config for copy-paste:

    widefield_odmr_logic:
        module.Class: 'widefield_odmr_logic.WidefieldOdmrLogic'
        connect:
            camera: camera_prime95b
            sync: widefield_sync
            microwave: mw_dummy
        options:
            mw_frequency: 2.87e9
            mw_power: -30
            enable_mw_output: False   # True ONLY with a dummy MW, or after SAFE-xxx exists
            exposure: 0.03            # seconds
            period_ms: 100
            default_n_pairs: 25
    """

    _camera = Connector(name='camera', interface='CameraInterface')
    _sync = Connector(name='sync', interface='WidefieldSync')
    _microwave = Connector(name='microwave', interface='MicrowaveInterface', optional=True)

    _mw_frequency = ConfigOption(name='mw_frequency', default=2.87e9)
    _mw_power = ConfigOption(name='mw_power', default=-30)
    _enable_mw_output = ConfigOption(name='enable_mw_output', default=False)
    _exposure = ConfigOption(name='exposure', default=0.03)
    _period_ms = ConfigOption(name='period_ms', default=100)
    _default_n_pairs = ConfigOption(name='default_n_pairs', default=25)
    # Diagnostic switch (frame-26 hunt, 2026-08-31): False = zero GUI emissions
    # during the poll loop — isolates the emission path from the acquisition path.
    _live_updates = ConfigOption(name='live_updates', default=True)

    sigPairDone = QtCore.Signal(int, int)                 # pairs done, pairs total
    sigImagesUpdated = QtCore.Signal(object, object, float)  # reference, contrast_img, mean_contrast
    sigBurstFinished = QtCore.Signal(bool, str)           # ok, message
    sigSweepPointDone = QtCore.Signal(int, int)           # M4: points done, points total
    sigSweepFinished = QtCore.Signal(bool, str)           # M4: ok, message
    _sigStartBurst = QtCore.Signal(int)                   # internal: run in logic thread
    _sigStartSweep = QtCore.Signal(float, float, int, int)  # internal (M4)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._thread_lock = RecursiveMutex()
        self._abort = False
        self._mean_on = None
        self._mean_off = None
        self._contrast_image = None
        # M4 sweep data
        self._sweep_frequencies = None       # 1D array [Hz]
        self._contrast_stack = None          # (n_points, H, W) float32, NaN = not measured
        self._sweep_reference = None         # mean MW-off image over the whole sweep

    def on_activate(self):
        self._abort = False
        self._sigStartBurst.connect(self._run_burst, QtCore.Qt.QueuedConnection)
        self._sigStartSweep.connect(self._run_sweep, QtCore.Qt.QueuedConnection)

    def on_deactivate(self):
        self.stop_burst()
        self._sigStartSweep.disconnect()
        self._sigStartBurst.disconnect()

    # ---------------- public API (GUI calls these) ----------------

    @property
    def images(self):
        """ (MW-off reference image, per-pixel contrast image) — the GUI's two panes. """
        return self._mean_off, self._contrast_image

    @property
    def default_n_pairs(self):
        return int(self._default_n_pairs)

    @property
    def sweep_frequencies(self):
        """ 1D array of sweep frequencies [Hz], or None. """
        return self._sweep_frequencies

    @property
    def contrast_stack(self):
        """ (n_points, H, W) float32 per-pixel contrast; NaN = point not measured yet. """
        return self._contrast_stack

    @property
    def sweep_reference(self):
        """ Mean MW-off image accumulated over the sweep, or None. """
        return self._sweep_reference

    def start_burst(self, n_pairs):
        """ Queue a burst into the logic thread (returns immediately). """
        self._sigStartBurst.emit(int(n_pairs))

    def start_sweep(self, f_start_hz, f_stop_hz, n_points, pairs_per_point):
        """ Queue an M4 frequency sweep into the logic thread (returns immediately). """
        self._sigStartSweep.emit(float(f_start_hz), float(f_stop_hz),
                                 int(n_points), int(pairs_per_point))

    def stop_burst(self):
        """ Abort an ongoing burst as fast as safely possible. """
        self._abort = True

    def set_exposure(self, exposure_s):
        with self._thread_lock:
            if self.module_state() != 'idle':
                self.log.error('Cannot set exposure during a burst.')
                return
            self._exposure = float(exposure_s)

    def set_period(self, period_ms):
        with self._thread_lock:
            if self.module_state() != 'idle':
                self.log.error('Cannot set period during a burst.')
                return
            self._period_ms = float(period_ms)

    # ---------------- burst implementation ----------------

    def _run_burst(self, n_pairs):
        with self._thread_lock:
            if self.module_state() != 'idle':
                self.log.error('Burst already running.')
                return
            self.module_state.lock()
        self._abort = False
        camera = self._camera()
        sync = self._sync()
        mw = self._microwave() if self._microwave.is_connected else None
        n_frames = 2 * n_pairs
        mw_on = False
        ok, msg = False, ''
        try:
            # Microwave: configure always; OUTPUT only if explicitly enabled by config.
            if mw is not None:
                mw.set_cw(float(self._mw_frequency), float(self._mw_power))
                if self._enable_mw_output:
                    mw.cw_on()
                    mw_on = True
                else:
                    self.log.info('MW output NOT enabled (enable_mw_output=False) — '
                                  'gated frames carry no RF.')

            camera.set_exposure_mode('Edge Trigger')  # verified mode name, M2.5 log
            camera.set_exposure(float(self._exposure))
            camera.start_frame_sequence(n_frames)
            sync.run_pair_burst(n_pairs, float(self._period_ms), gate_on_first=True)

            # Poll TIGHT, process cheap, update the GUI THROTTLED: heavy per-pair work
            # in this loop makes polling lag and PVCAM frame notifications get missed
            # ("Frame timeout" mid-burst — seen at the first M3 GUI run, 50 frames).
            sum_on = None
            sum_off = None
            first_count = None
            last_emit = 0.0  # TIME-based GUI throttle (a pair-count throttle was a
            #                  no-op at small n_pairs — first M3 run stalled ~frame 26)
            frame_times = []  # diagnostic: per-frame arrival stamps (frame-26 hunt)
            for i in range(n_frames):
                if self._abort:
                    raise RuntimeError('Aborted by user.')
                try:
                    frame, count = camera.poll_next_frame(
                        timeout_ms=int(5 * self._period_ms) + 5000)
                    frame_times.append(time.monotonic())
                except Exception as e:
                    if self._abort:
                        raise RuntimeError('Aborted by user.') from e
                    if len(frame_times) > 1:
                        gaps = np.diff(frame_times[-8:])
                        self.log.error(
                            f'Frame-timing before the stall (last intervals, s): '
                            f'{np.array2string(gaps, precision=3)}')
                    raise RuntimeError(f'frame {i + 1}/{n_frames}: {e}') from e
                if first_count is None:
                    first_count = count
                elif count != first_count + i:
                    raise RuntimeError(
                        f'PARITY GUARD: frame counter gap at frame {i} '
                        f'(expected {first_count + i}, got {count}) — on/off roles '
                        f'would be untrustworthy. Burst aborted.')
                if i % 2 == 0:   # gate_on_first=True -> even index = gated ("RF on")
                    if sum_on is None:
                        sum_on = frame.astype(np.float32)
                    else:
                        np.add(sum_on, frame, out=sum_on)
                else:
                    if sum_off is None:
                        sum_off = frame.astype(np.float32)
                    else:
                        np.add(sum_off, frame, out=sum_off)
                    pairs_done = (i + 1) // 2
                    if (self._live_updates and (time.monotonic() - last_emit) > 1.0) \
                            or pairs_done == n_pairs:
                        last_emit = time.monotonic()
                        self._mean_on = sum_on / pairs_done
                        self._mean_off = sum_off / pairs_done
                        # Per-pixel contrast map (operator layout ruling 2026-08-31):
                        # left = MW-off reference, right = (on - off) / off.
                        self._contrast_image = np.divide(
                            self._mean_on - self._mean_off, self._mean_off,
                            out=np.zeros_like(self._mean_off),
                            where=self._mean_off != 0)
                        contrast = self._mean_contrast(self._mean_on, self._mean_off)
                        self.sigPairDone.emit(pairs_done, n_pairs)
                        self.sigImagesUpdated.emit(self._mean_off, self._contrast_image,
                                                   contrast)

            # Parity guard, second half: no extra frame may exist.
            extra = False
            try:
                camera.poll_next_frame(timeout_ms=int(2 * self._period_ms))
                extra = True
            except Exception:
                pass
            if extra:
                raise RuntimeError('PARITY GUARD: extra frame after burst — '
                                   'trigger/frame count mismatch.')
            ok, msg = True, f'{n_pairs} pairs acquired, parity clean.'
        except Exception as err:
            ok, msg = False, str(err)
            self.log.error(f'Burst failed: {err}')
        finally:
            try:
                camera.finish_sequence()
            except Exception:
                pass
            try:
                sync.all_low()
            except Exception:
                pass
            if mw_on:
                try:
                    mw.off()
                except Exception:
                    self.log.exception('MW off failed:')
            self.module_state.unlock()
            self.sigBurstFinished.emit(ok, msg)

    # ---------------- M4 frequency sweep ----------------

    def _acquire_pair_sums(self, camera, sync, n_pairs):
        """ Tight acquisition of n_pairs on/off pairs with the parity guard; NO GUI
        emissions (the sweep emits once per frequency point, not per frame).

        @return tuple: (sum_on, sum_off) float32 arrays
        """
        n_frames = 2 * n_pairs
        camera.start_frame_sequence(n_frames)
        sync.run_pair_burst(n_pairs, float(self._period_ms), gate_on_first=True)
        sum_on = None
        sum_off = None
        first_count = None
        for i in range(n_frames):
            if self._abort:
                raise RuntimeError('Aborted by user.')
            try:
                frame, count = camera.poll_next_frame(
                    timeout_ms=int(5 * self._period_ms) + 5000)
            except Exception as e:
                if self._abort:
                    raise RuntimeError('Aborted by user.') from e
                raise RuntimeError(f'frame {i + 1}/{n_frames}: {e}') from e
            if first_count is None:
                first_count = count
            elif count != first_count + i:
                raise RuntimeError(
                    f'PARITY GUARD: frame counter gap at frame {i} '
                    f'(expected {first_count + i}, got {count}).')
            if i % 2 == 0:
                if sum_on is None:
                    sum_on = frame.astype(np.float32)
                else:
                    np.add(sum_on, frame, out=sum_on)
            else:
                if sum_off is None:
                    sum_off = frame.astype(np.float32)
                else:
                    np.add(sum_off, frame, out=sum_off)
        extra = False
        try:
            camera.poll_next_frame(timeout_ms=int(2 * self._period_ms))
            extra = True
        except Exception:
            pass
        camera.finish_sequence()
        if extra:
            raise RuntimeError('PARITY GUARD: extra frame after burst.')
        return sum_on, sum_off

    def _run_sweep(self, f_start_hz, f_stop_hz, n_points, pairs_per_point):
        with self._thread_lock:
            if self.module_state() != 'idle':
                self.log.error('Sweep/burst already running.')
                return
            self.module_state.lock()
        self._abort = False
        camera = self._camera()
        sync = self._sync()
        mw = self._microwave() if self._microwave.is_connected else None
        mw_on = False
        ok, msg = False, ''
        try:
            height, width = camera.get_size()[1], camera.get_size()[0]
            est_bytes = n_points * height * width * 4
            if est_bytes > 800e6:
                raise RuntimeError(
                    f'Sweep stack would need {est_bytes / 1e6:.0f} MB (> 800 MB cap): '
                    f'reduce points, or use a camera ROI (M4 follow-up).')
            freqs = np.linspace(float(f_start_hz), float(f_stop_hz), int(n_points))
            self._sweep_frequencies = freqs
            self._contrast_stack = np.full((n_points, height, width), np.nan,
                                           dtype=np.float32)
            self._sweep_reference = None
            camera.set_exposure_mode('Edge Trigger')
            camera.set_exposure(float(self._exposure))
            off_accum = None
            for k, f in enumerate(freqs):
                if self._abort:
                    raise RuntimeError('Aborted by user.')
                if mw is not None:
                    # Software-stepped (mandate/design O2): retune between points.
                    if mw_on:
                        mw.off()
                        mw_on = False
                    mw.set_cw(float(f), float(self._mw_power))
                    if self._enable_mw_output:
                        mw.cw_on()
                        mw_on = True
                sum_on, sum_off = self._acquire_pair_sums(camera, sync, pairs_per_point)
                mean_on = sum_on / pairs_per_point
                mean_off = sum_off / pairs_per_point
                self._contrast_stack[k] = np.divide(
                    mean_on - mean_off, mean_off,
                    out=np.zeros_like(mean_off), where=mean_off != 0)
                off_accum = mean_off if off_accum is None else off_accum + mean_off
                self._sweep_reference = off_accum / (k + 1)
                self.sigSweepPointDone.emit(k + 1, n_points)
            ok, msg = True, f'{n_points} points x {pairs_per_point} pairs, parity clean.'
        except Exception as err:
            ok, msg = False, str(err)
            self.log.error(f'Sweep failed: {err}')
        finally:
            try:
                camera.finish_sequence()
            except Exception:
                pass
            try:
                sync.all_low()
            except Exception:
                pass
            if mw_on:
                try:
                    mw.off()
                except Exception:
                    self.log.exception('MW off failed:')
            self.module_state.unlock()
            self.sigSweepFinished.emit(ok, msg)

    @staticmethod
    def _mean_contrast(mean_on, mean_off):
        """ Mean relative difference (on - off) / off over all pixels. """
        denom = np.mean(mean_off)
        if denom == 0:
            return 0.0
        return float((np.mean(mean_on) - denom) / denom)
