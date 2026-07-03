# -*- coding: utf-8 -*-

"""
Qudi hardware module: continuous photon-counting data in-streamer for a Swabian Time Tagger.

Implements DataInStreamInterface on top of TimeTagger.Counter so the qudi Time Series
toolchain (time_series_reader_logic + time_series_gui) can display a live count(-rate) trace
from the APD click channels.

SAFETY: this module is read-only with respect to the experiment — it only counts incoming TTL
ticks on the configured Time Tagger input channels. It enables no laser, RF, or motion.

STATUS: first draft for THIS setup (confocal_odmr). No upstream Time Tagger streamer exists, so
this is custom code. The qudi-interface side mirrors hardware/dummy/data_instream_dummy.py. The
Time-Tagger-specific parts marked `# VALIDATE:` need confirmation against the Time Tagger docs
and a live test, and the measurement correctness (counts vs rate, bin alignment, units) needs
human/expert validation before the data is trusted. Recommended for Codex review.

Copyright (c) 2026, the qudi developers.
This file is part of qudi. Licensed under LGPL v3 (see qudi for details).
"""

import time
import numpy as np
from typing import List, Optional, Sequence, Tuple, Union

import TimeTagger as tt

from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector
from qudi.util.constraints import ScalarConstraint
from qudi.util.mutex import Mutex
from qudi.interface.data_instream_interface import (DataInStreamInterface, DataInStreamConstraints,
                                                    StreamingMode, SampleTiming)


class TimeTaggerInstreamer(DataInStreamInterface):
    """
    Continuous photon-counting in-streamer for a Swabian Time Tagger.

    Example config:

    timetagger_instreamer:
        module.Class: 'swabian_instruments.timetagger_instreamer.TimeTaggerInstreamer'
        options:
            channels:           # qudi channel name -> Time Tagger input channel number
                APD1: 1
                APD2: 2
            # serial: '1740000JEC'   # optional; omit to auto-connect the only Time Tagger
            # sample_rate: 50.0       # Hz (bin rate), default
            # buffer_size: 1048576    # samples per channel (rolling counter depth), default
            # count_rate: True        # True -> counts/s per bin; False -> raw counts/bin
            # max_sample_rate: 1.0e6  # Hz; to_be_confirmed vs Time Tagger 20 specs
    """
    _channel_config = ConfigOption(name='channels', missing='error')
    _serial = ConfigOption(name='serial', default='', missing='nothing')
    _default_sample_rate = ConfigOption(name='sample_rate', default=50.0)
    _default_buffer_size = ConfigOption(name='buffer_size', default=1024 ** 2,
                                        constructor=lambda x: int(x))
    _report_count_rate = ConfigOption(name='count_rate', default=True)
    # photon counts (and integer-Hz rates) are integer-valued -> integer dtype gives a clean
    # integer display in the GUI (no spurious decimals). Override to 'float64' if ever needed.
    _data_type = ConfigOption(name='data_type', default='int64',
                              constructor=lambda t: np.dtype(t).type)
    # VALIDATE: Time Tagger 20 max meaningful bin rate — placeholder cap, confirm vs specs.
    _max_sample_rate = ConfigOption(name='max_sample_rate', default=1.0e6)
    # Optional: borrow the Time Tagger object from the dedicated owner module (timetagger_provider),
    # so the live counter and the confocal scan / ODMR sweep all share ONE Time Tagger connection.
    # If not connected, this module opens its own (standalone counter use).
    _tagger_provider = Connector(name='tagger_provider', interface='TimeTaggerProvider',
                                 optional=True)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._thread_lock = Mutex()
        self._tagger = None
        self._owns_tagger = False
        self._counter = None
        self._constraints = None
        self._channel_names = []
        self._channel_numbers = []
        self._active_channels = []
        self._sample_rate = 0.0
        self._buffer_size = 0
        self._streaming_mode = StreamingMode.CONTINUOUS
        self._bin_width_ps = 0
        self._consumed_bins = 0

    # ----- qudi lifecycle -------------------------------------------------------------------
    def on_activate(self):
        self._channel_names = [str(name) for name in self._channel_config.keys()]
        self._channel_numbers = [int(ch) for ch in self._channel_config.values()]
        if len(set(self._channel_names)) != len(self._channel_names):
            raise ValueError('Duplicate channel names in "channels" ConfigOption')

        # Use a shared Time Tagger if a provider module is connected, else open our own (read-only).
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
            self._tagger = tt.createTimeTagger(self._serial) if self._serial else tt.createTimeTagger()
            self._owns_tagger = True
            self.log.info(f'Connected own Time Tagger, serial {self._tagger.getSerial()}.')

        unit = 'c/s' if self._report_count_rate else 'counts'
        self._constraints = DataInStreamConstraints(
            channel_units={name: unit for name in self._channel_names},
            sample_timing=SampleTiming.CONSTANT,
            streaming_modes=[StreamingMode.CONTINUOUS, StreamingMode.FINITE],
            data_type=self._data_type,
            channel_buffer_size=ScalarConstraint(default=int(self._default_buffer_size),
                                                 bounds=(128, 1024 ** 3),
                                                 increment=1,
                                                 enforce_int=True),
            sample_rate=ScalarConstraint(default=float(self._default_sample_rate),
                                         bounds=(0.1, float(self._max_sample_rate)),
                                         increment=0.1),
        )
        self._active_channels = list(self._channel_names)
        self._sample_rate = float(self._default_sample_rate)
        self._buffer_size = int(self._default_buffer_size)
        self._streaming_mode = StreamingMode.CONTINUOUS

    def on_deactivate(self):
        try:
            self.stop_stream()
        except Exception:
            pass
        self._counter = None
        # Only free the device if we opened it ourselves (a borrowed tagger is owned elsewhere).
        if self._owns_tagger and self._tagger is not None:
            tt.freeTimeTagger(self._tagger)
        self._tagger = None

    # ----- read-only properties -------------------------------------------------------------
    @property
    def constraints(self) -> DataInStreamConstraints:
        return self._constraints

    @property
    def sample_rate(self) -> float:
        return self._sample_rate

    @property
    def channel_buffer_size(self) -> int:
        return self._buffer_size

    @property
    def streaming_mode(self) -> StreamingMode:
        return self._streaming_mode

    @property
    def active_channels(self) -> List[str]:
        return self._active_channels.copy()

    @property
    def available_samples(self) -> int:
        with self._thread_lock:
            if self.module_state() != 'locked' or self._counter is None:
                return 0
            return self._available_unlocked()

    # ----- configuration --------------------------------------------------------------------
    def configure(self,
                  active_channels: Sequence[str],
                  streaming_mode: Union[StreamingMode, int],
                  channel_buffer_size: int,
                  sample_rate: float) -> None:
        with self._thread_lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Unable to configure Time Tagger stream while it is running')

            channels = list(active_channels)
            if not set(channels).issubset(self._channel_names):
                raise ValueError(f'Invalid channels {channels}; allowed: {self._channel_names}')

            try:
                mode = StreamingMode(streaming_mode.value)
            except AttributeError:
                mode = StreamingMode(streaming_mode)
            if mode not in self._constraints.streaming_modes:
                raise ValueError(f'Invalid streaming mode {mode}')

            self._constraints.channel_buffer_size.check(channel_buffer_size)
            self._constraints.sample_rate.check(sample_rate)

            self._active_channels = channels
            self._streaming_mode = mode
            self._buffer_size = int(channel_buffer_size)
            self._sample_rate = float(sample_rate)

    # ----- stream control -------------------------------------------------------------------
    def start_stream(self) -> None:
        with self._thread_lock:
            if self.module_state() != 'idle':
                self.log.warning('Time Tagger stream already running.')
                return
            self.module_state.lock()
            try:
                # one bin == one sample
                self._bin_width_ps = int(round(1e12 / self._sample_rate))
                active_numbers = [self._channel_numbers[self._channel_names.index(n)]
                                  for n in self._active_channels]
                # VALIDATE: TimeTagger.Counter is a rolling buffer of the last n_values bins of
                # width binwidth (ps). Confirm bin alignment / partial-bin behavior vs the docs.
                self._counter = tt.Counter(self._tagger,
                                           channels=active_numbers,
                                           binwidth=self._bin_width_ps,
                                           n_values=self._buffer_size)
                self._consumed_bins = 0
            except Exception:
                self.module_state.unlock()
                self._counter = None
                raise

    def stop_stream(self) -> None:
        with self._thread_lock:
            if self.module_state() == 'locked':
                if self._counter is not None:
                    self._counter.stop()
                self._counter = None
                self.module_state.unlock()

    # ----- internal helpers -----------------------------------------------------------------
    def _available_unlocked(self) -> int:
        # VALIDATE: getCaptureDuration() unit assumed to be picoseconds.
        total_bins = int(self._counter.getCaptureDuration() // self._bin_width_ps)
        available = total_bins - self._consumed_bins
        if available < 0:
            available = 0
        if available > self._buffer_size:
            self.log.warning('Time Tagger counter buffer overflow — decrease sample rate or read '
                             'faster. Oldest samples were dropped.')
            self._consumed_bins = total_bins - self._buffer_size
            available = self._buffer_size
        return available

    def _read_block_unlocked(self, samples_per_channel: int) -> np.ndarray:
        """ Return interleaved (sample-major) 1D float64 array of the oldest unconsumed samples. """
        available = self._available_unlocked()
        k = min(samples_per_channel, available)
        if k <= 0:
            return np.empty(0, dtype=np.float64)
        data = np.asarray(self._counter.getData())  # shape (n_channels, n_values), counts per bin
        # unconsumed bins are the newest `available` columns; take the oldest k of those (FIFO)
        start = self._buffer_size - available
        block = data[:, start:start + k].astype(np.float64)
        if self._report_count_rate:
            block = block / (self._bin_width_ps * 1e-12)  # counts -> counts/s
        self._consumed_bins += k
        # photon counts/rates are integer-valued; round before casting to an integer dtype so the
        # GUI shows clean integers (e.g. "500 c/s", not "500.00000")
        if np.issubdtype(self._data_type, np.integer):
            block = np.rint(block)
        block = block.astype(self._data_type)
        # interleave sample-major: flat[s*n_ch + c] = block[c, s]
        return block.T.reshape(-1)

    def _wait_for_samples(self, samples_per_channel: int) -> None:
        available = self._available_unlocked()
        while available < samples_per_channel:
            time.sleep(max((samples_per_channel - available) / self._sample_rate, 1e-3))
            available = self._available_unlocked()

    # ----- read methods ---------------------------------------------------------------------
    def read_data_into_buffer(self,
                              data_buffer: np.ndarray,
                              samples_per_channel: int,
                              timestamp_buffer: Optional[np.ndarray] = None) -> None:
        with self._thread_lock:
            if self.module_state() != 'locked':
                raise RuntimeError('Unable to read data. Stream is not running.')
            ch_count = len(self._active_channels)
            if data_buffer.size < samples_per_channel * ch_count:
                raise RuntimeError(f'data_buffer too small ({data_buffer.size}) for '
                                   f'{ch_count} x {samples_per_channel} samples')
            self._wait_for_samples(samples_per_channel)
            flat = self._read_block_unlocked(samples_per_channel)
            data_buffer[:flat.size] = flat

    def read_available_data_into_buffer(self,
                                        data_buffer: np.ndarray,
                                        timestamp_buffer: Optional[np.ndarray] = None) -> int:
        with self._thread_lock:
            if self.module_state() != 'locked':
                raise RuntimeError('Unable to read data. Stream is not running.')
            ch_count = len(self._active_channels)
            max_samples = data_buffer.size // ch_count
            available = min(self._available_unlocked(), max_samples)
            if available <= 0:
                return 0
            flat = self._read_block_unlocked(available)
            data_buffer[:flat.size] = flat
            return flat.size // ch_count

    def read_data(self,
                  samples_per_channel: Optional[int] = None
                  ) -> Tuple[np.ndarray, Union[np.ndarray, None]]:
        with self._thread_lock:
            if self.module_state() != 'locked':
                raise RuntimeError('Unable to read data. Stream is not running.')
            if samples_per_channel is None:
                samples_per_channel = self._available_unlocked()
            else:
                self._wait_for_samples(samples_per_channel)
            flat = self._read_block_unlocked(samples_per_channel)
            return flat, None

    def read_single_point(self) -> Tuple[np.ndarray, Union[None, np.float64]]:
        with self._thread_lock:
            if self.module_state() != 'locked':
                raise RuntimeError('Unable to read data. Stream is not running.')
            self._wait_for_samples(1)
            flat = self._read_block_unlocked(1)
            return flat, None
