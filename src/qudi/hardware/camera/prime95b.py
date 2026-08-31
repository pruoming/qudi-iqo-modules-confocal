# -*- coding: utf-8 -*-

"""
Hardware module for the Teledyne Photometrics Prime 95B camera via PyVCAM/PVCAM.

New-generation qudi port (widefield_odmr setup, 2026-08-31) of a legacy old-generation
qudi module by Srihari Jayaram (github.com/sriharijayaram5/qudi, branch Prime95B),
archived verbatim in the setup's records
(Qudi_AI/setups/widefield_odmr/knowledge/old_setup_references/prime95b.py).
Port ledger entry: Qudi_AI/setups/widefield_odmr/knowledge/fork_changes.yaml.

Key deviations from the legacy module (design doc §4):
- The camera is opened BY NAME (ConfigOption ``camera_name``) — NEVER open-first-found:
  this bench cables the camera over USB and PCIe simultaneously, it enumerates twice,
  and blind-opening the first enumeration freezes uninterruptibly inside the vendor
  DLL (widefield_odmr known issue CAM-001).
- Exposure is in SECONDS at the interface (new CameraInterface contract); the legacy
  module used PVCAM's native milliseconds.
- Trigger/readout settings are ConfigOptions, not hard-coded. ``exp_out_mode`` defaults
  to None (NOT SET): the all-rows/global-exposure choice is design open item O7 and must
  not be assumed.

The legacy source is GPLv3 (old qudi); this adaptation remains under the GNU General
Public License v3 accordingly.
"""

import numpy as np

from qudi.core.configoption import ConfigOption
from qudi.interface.camera_interface import CameraInterface

from pyvcam import pvc
from pyvcam.camera import Camera
from pyvcam import constants as const


class Prime95B(CameraInterface):
    """ Hardware class for the Photometrics Prime 95B via PyVCAM.

    Example config for copy-paste:

    camera_prime95b:
        module.Class: 'camera.prime95b.Prime95B'
        options:
            camera_name: 'PMPCIECam00'   # open BY NAME - never first-found (CAM-001)
            exposure_mode: 'Internal Trigger'   # e.g. 'Ext Trig Internal' for triggered frames
            speed_table_index: 1         # 1 = 16-bit mode (legacy-validated)
            # exp_out_mode: null         # design O7: set only once the all-rows mode is confirmed
            exposure: 0.01               # seconds
            gain: 1
    """

    _camera_name = ConfigOption(name='camera_name', default='PMPCIECam00', missing='warn')
    _exposure_mode = ConfigOption(name='exposure_mode', default='Internal Trigger', missing='info')
    _speed_table_index = ConfigOption(name='speed_table_index', default=1, missing='info')
    _exp_out_mode = ConfigOption(name='exp_out_mode', default=None)  # None = leave camera default (O7)
    _default_exposure = ConfigOption(name='exposure', default=0.01)  # seconds
    _default_gain = ConfigOption(name='gain', default=1)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cam = None
        self._live = False

    # ---------------- lifecycle ----------------

    def on_activate(self):
        pvc.init_pvcam()
        try:
            names = [pvc.get_cam_name(i) for i in range(pvc.get_cam_total())]
            if self._camera_name not in names:
                raise RuntimeError(
                    f'Camera "{self._camera_name}" not enumerated. Found: {names}. '
                    f'Check the PCIe link/power (CAM-001: never open first-found).')
            for c in Camera.detect_camera():  # yields without opening
                if c.name == self._camera_name:
                    self.cam = c
                    break
            self.cam.open()
            # Legacy-validated readout defaults, now config-driven:
            self.cam.speed_table_index = int(self._speed_table_index)
            self.cam.exp_mode = self._exposure_mode
            if self._exp_out_mode is not None:
                self.cam.exp_out_mode = int(self._exp_out_mode)
            self.cam.exp_res = 0  # PVCAM exposure unit: milliseconds (conversion below)
            self.set_exposure(float(self._default_exposure))
            self.set_gain(self._default_gain)
            self._live = False
        except Exception:
            # Leave no half-initialized PVCAM state behind (a zombie handle freezes retries).
            self._shut_down()
            raise

    def on_deactivate(self):
        self.stop_acquisition()
        self._shut_down()

    def _shut_down(self):
        try:
            if self.cam is not None and self.cam.is_open:
                self.cam.close()
        except Exception:
            self.log.exception('Camera close failed:')
        finally:
            self.cam = None
            try:
                pvc.uninit_pvcam()
            except Exception:
                pass

    # ---------------- CameraInterface ----------------

    def get_name(self):
        """ @return string: name of the camera """
        return self.cam.name

    def get_size(self):
        """ @return tuple: current image size (width, height) — ROI-dependent """
        return self.cam.shape()  # PyVCAM 2.x: shape(roi_index=0) is a method

    def support_live_acquisition(self):
        """ @return bool """
        return True

    def start_live_acquisition(self):
        """ Start continuous acquisition. @return bool

        Always resets the exposure mode to the configured free-running default first:
        a preceding widefield burst leaves the camera in 'Edge Trigger', and live view
        would then block forever waiting for triggers that never come.
        """
        self.cam.exp_mode = self._exposure_mode
        self.cam.start_live()
        self._live = True
        return True

    def stop_acquisition(self):
        """ Stop/abort live or single acquisition. @return bool """
        if self._live:
            self._live = False
            self.cam.finish()  # PyVCAM 2.x: finish() ends live/sequence (stop_live is gone)
        return True

    def start_single_acquisition(self):
        """ Single acquisition; the frame is fetched in get_acquired_data(). @return bool """
        return True

    def get_acquired_data(self):
        """ @return numpy array: last acquired frame [[row], [row], ...] """
        if self._live:
            # PyVCAM 2.x live API: poll_frame() -> (frame_dict, fps, frame_count).
            # Finite timeout — the default waits FOREVER and freezes the logic thread
            # if no frame ever comes (e.g. a triggered mode with no trigger source).
            frame, _fps, _count = self.cam.poll_frame(timeout_ms=10000)
            return frame['pixel_data']
        return self.cam.get_frame()

    def set_exposure(self, exposure):
        """ Set exposure time in SECONDS (interface contract; PVCAM works in ms here).

        @param float exposure: seconds; resolution 1 ms, minimum 1 ms in this mode
        @return float: actually set exposure in seconds
        """
        ms = max(1, int(round(float(exposure) * 1000)))
        self.cam.exp_time = ms
        return self.get_exposure()

    def get_exposure(self):
        """ @return float: exposure time in SECONDS """
        exp_res_divisor = {0: 1e3, 1: 1e6, 2: 1.0}  # index: units per second (0=ms, 1=us, 2=s)
        return self.cam.exp_time / exp_res_divisor[self.cam.exp_res_index]

    def set_gain(self, gain):
        """ @return float: new gain """
        self.cam.gain = int(gain)
        return self.cam.gain

    def get_gain(self):
        """ @return float: gain """
        return self.cam.gain

    def get_ready_state(self):
        """ @return bool """
        return bool(self.cam is not None and self.cam.is_open and not self._live)

    # ------------- extensions beyond CameraInterface (design §4; used from M2.5/M3 logic) -------------

    def get_sensor_size(self):
        """ @return tuple: full sensor size (width, height), ROI-independent """
        return self.cam.sensor_size

    def set_exposure_mode(self, exp_mode):
        """ Set trigger behavior (e.g. 'Internal Trigger', 'Ext Trig Internal'). """
        self.cam.exp_mode = exp_mode
        return True

    def get_exposure_mode(self):
        return self.cam.exp_mode

    def available_exposure_modes(self):
        """ @return dict: valid exposure-mode names for this camera """
        return self.cam.read_enum(const.PARAM_EXPOSURE_MODE)

    def available_exp_out_modes(self):
        """ @return dict: valid expose-out mode names (design O7 check reads this) """
        return self.cam.read_enum(const.PARAM_EXPOSE_OUT_MODE)

    def set_exp_out_mode(self, mode):
        """ Set the EXPOSE OUT behavior (all-rows window etc. — design O7). """
        self.cam.exp_out_mode = mode
        return True

    def set_speed_index(self, index):
        """ ADC mode: 16-bit vs 12-bit (affects allowed gain range). """
        count = self.cam.get_param(const.PARAM_SPDTAB_INDEX, const.ATTR_COUNT)
        if int(index) >= count:
            raise ValueError(f'{self.cam.name} only supports speed indices < {count}.')
        self.cam.speed_table_index = int(index)
        return True

    def get_max_gain(self):
        return self.cam.get_param(const.PARAM_GAIN_INDEX, const.ATTR_MAX)

    def get_max_exposure(self):
        """ @return float: maximum exposure in current PVCAM resolution units """
        return self.cam.get_param(const.PARAM_EXPOSURE_TIME, const.ATTR_MAX)

    def start_frame_sequence(self, num_frames):
        """ Arm a triggered acquisition WITHOUT blocking (poll frames afterwards).
        Caller contract: exactly num_frames triggers will arrive (design §3 parity guard).

        Implementation note (2026-08-31): uses LIVE/circular-buffer mode, not
        start_seq — finite-sequence acquisitions died deterministically at frame 26
        of 50 (suspected PVCAM sequence-buffer cap; M2.5's 20 frames passed). The
        ring (up to 64 frames deep) queues triggered frames even if the consumer
        lags; poll_next_frame(oldestFrame) drains it in order.
        """
        if not self.get_ready_state():
            raise RuntimeError('Camera not ready to arm a sequence (open? live running?).')
        buffer_frames = int(max(16, min(int(num_frames), 64)))
        self.cam.start_live(buffer_frame_count=buffer_frames)
        return True

    def poll_next_frame(self, timeout_ms=15000):
        """ Fetch the next frame of an armed sequence.

        @return tuple: (2D ndarray pixel data, int frame_count from the camera)
        Raises on timeout — a missing trigger must fail loudly, never silently.
        """
        frame, _fps, count = self.cam.poll_frame(timeout_ms=int(timeout_ms))
        return frame['pixel_data'], count

    def finish_sequence(self):
        """ End an armed/running sequence acquisition. """
        self.cam.finish()
        return True

    def get_sequence(self, num_frames):
        """ Blocking finite acquisition of num_frames frames (one per trigger in
        externally triggered modes) — the M2.5/M3 workhorse. Frame parity is the
        caller's contract: trigger count MUST equal num_frames (design §3).

        @return ndarray: (num_frames, height, width)
        """
        if not self.get_ready_state():
            raise RuntimeError('Camera not ready for sequence acquisition (open? live running?).')
        return self.cam.get_sequence(int(num_frames))

    def set_roi(self, x_start, y_start, width, height):
        """ Single rectangular ROI in sensor pixels (ints).

        PyVCAM 2.x signature: set_roi(s1, p1, w, h); reset first so exactly one ROI exists.
        (The legacy module assigned a (x1, x2, y1, y2) tuple to cam.roi — API removed.)
        """
        self.cam.reset_rois()
        self.cam.set_roi(int(x_start), int(y_start), int(width), int(height))
        return True
