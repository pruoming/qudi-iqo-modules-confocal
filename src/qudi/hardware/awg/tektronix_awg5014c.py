# -*- coding: utf-8 -*-

"""
Qudi hardware module for the Tektronix AWG5014C arbitrary waveform generator.

FORK MODULE (confocal_odmr, cycle 2, AWG intake phase B — 2026-07-22).
Adapted from the legacy-interface donor tektronix_awg5002c.py (same AWG5000 series,
SCPI + .wfm + FTP architecture) and ported to the CURRENT PulserInterface following
the tektronix_awg70k.py patterns (pyvisa transport, write_waveform/load_waveform API).

PHASE-B SCOPE (mock-first; no instrument contact):
  * SCPI control via VISA/GPIB (NI GPIB-USB-HS; resource per probe_awg5014c_idn.py).
  * Waveform files (.wfm) are written locally; upload uses the donor's FTP path but the
    FTP address is a SEPARATE config option, default None => upload DISABLED with a loud
    error (the ethernet link is phase C, connections.yaml devices.awg to_be_confirmed).
  * SEQUENCE MODE IS OUT OF SCOPE (non-goal this phase): sequence_option = NON,
    write/load_sequence fail loudly. Planned for phase D+.

CONSTRAINTS ARE PARAMETERIZED, NOT SOURCED (REQ-028 open when this was written):
  shared_knowledge/device_models.md has NO AWG5014C entry yet. All 5014C-specific limits
  below are ConfigOptions with deliberately CONSERVATIVE (understating) defaults and must
  be reconciled against the datasheet-sourced device_models entry when REQ-028 is answered:
    sample_rate_max      default 600.0e6  (donor 5002C value; 5014C spec is believed higher)
    min_waveform_length  default 250      (conservative family guess)
    waveform_granularity default 4        (conservative: rejects lengths a granularity-1
                                           instrument would accept — safe direction)
    max_waveform_length  default 4_000_000 (conservative; well under any 5014C memory size)
  Family values kept from the donor (amplitude/offset/marker ranges, sample_rate min)
  are marked TBC(REQ-028) inline.

SOURCED PROTOCOL FACTS (AWG5000/7000 Series Programmer Manual 077006105 Rev A, partial
extract fetched 2026-07-22 -> setups/confocal_odmr/hardware_manuals/):
  * AWGControl:RSTate? returns 0=stopped, 1=waiting-for-trigger, 2=running (p. 2-39);
    qudi status ints are 0=stopped, 1=running, 2=waiting => mapping in get_status().
  * [SOURce[n]]:FUNCtion:USER loads a waveform/sequence file into a channel (5000 series).
  * AWGControl:RMODe {CONTinuous|TRIGgered|GATed|SEQuence} (p. 2-37).
  * MMEMory:IMPort supports type WFM (legacy AWG400-700 waveform format).
The .wfm BINARY LAYOUT used here ('MAGIC 1000\\r\\n' + '#<ndigits><nbytes>' + per-sample
[float32-LE analog, 1 marker byte with bit6=marker1, bit7=marker2] + 'CLOCK <rate>\\r\\n')
is the family-standard layout (donor lineage + Tek docs); the layout page of the manual is
NOT in the persisted extract => byte layout is TBC(REQ-028) and must be confirmed against
the full manual before phase-C hardware upload. Validation in phase B is structure-level.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory
of this distribution and on <https://github.com/Ulm-IQO/qudi-iqo-modules/>

This file is part of qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with qudi.
If not, see <https://www.gnu.org/licenses/>.
"""

import os
import time
from collections import OrderedDict
from fnmatch import fnmatch
from ftplib import FTP

import numpy as np

try:
    import pyvisa as visa
except ImportError:
    import visa

from qudi.core.configoption import ConfigOption
from qudi.util.helpers import natural_sort
from qudi.util.paths import get_appdata_dir
from qudi.interface.pulser_interface import PulserInterface, PulserConstraints, SequenceOption


class AWG5014C(PulserInterface):
    """ Tektronix AWG5014C: 4 analog channels, 2 markers per channel (d_ch1..d_ch8).

    Marker mapping (fixed by hardware topology):
        a_ch<n>  <->  markers d_ch<2n-1> (marker 1) and d_ch<2n> (marker 2)
        i.e. a_ch1: d_ch1/d_ch2, a_ch2: d_ch3/d_ch4, a_ch3: d_ch5/d_ch6, a_ch4: d_ch7/d_ch8

    Example config for copy-paste (phase B — no ethernet, upload disabled):

    pulser_awg5014c:
        module.Class: 'awg.tektronix_awg5014c.AWG5014C'
        options:
            awg_visa_address: 'GPIB0::1::INSTR'   # to_be_confirmed (REQ-028 / front panel)
            timeout: 10                            # VISA timeout in seconds
            # ftp_ip_address: '10.42.0.211'  # phase C: enables the FTP upload path
            # ftp_root_dir: 'C:\\inetpub\\ftproot'
            # ftp_login: 'anonymous'
            # ftp_passwd: 'anonymous@'
            # tmp_work_dir: 'C:\\Software\\qudi_pulsed_files'
            # default_sample_rate: 600.0e6   # applied on activation ONLY if set
            # --- provisional constraints, reconcile with device_models.md (REQ-028) ---
            # sample_rate_max: 600.0e6
            # min_waveform_length: 250
            # waveform_granularity: 4
            # max_waveform_length: 4000000
    """

    # ---- transport config ----
    _visa_address = ConfigOption(name='awg_visa_address', missing='error')
    _visa_timeout = ConfigOption(name='timeout', default=10, missing='warn')  # seconds
    # FTP upload path (donor architecture) — address parameterized; None = phase B, no upload
    _ftp_ip_address = ConfigOption(name='ftp_ip_address', default=None, missing='nothing')
    _ftp_root_dir = ConfigOption(name='ftp_root_dir', default='C:\\inetpub\\ftproot',
                                 missing='nothing')
    _username = ConfigOption(name='ftp_login', default='anonymous', missing='nothing')
    _password = ConfigOption(name='ftp_passwd', default='anonymous@', missing='nothing')
    _tmp_work_dir = ConfigOption(name='tmp_work_dir',
                                 default=os.path.join(get_appdata_dir(True), 'pulsed_files'),
                                 missing='warn')
    _default_sample_rate = ConfigOption(name='default_sample_rate', default=None,
                                        missing='nothing')

    # ---- provisional device constraints — TBC(REQ-028), conservative defaults ----
    _cfg_sample_rate_min = ConfigOption(name='sample_rate_min', default=10.0e6,
                                        missing='nothing')
    _cfg_sample_rate_max = ConfigOption(name='sample_rate_max', default=600.0e6,
                                        missing='nothing')
    _cfg_min_wfm_length = ConfigOption(name='min_waveform_length', default=250,
                                       missing='nothing')
    _cfg_wfm_granularity = ConfigOption(name='waveform_granularity', default=4,
                                        missing='nothing')
    _cfg_max_wfm_length = ConfigOption(name='max_waveform_length', default=4_000_000,
                                       missing='nothing')

    # channel topology (structural: 4 analog channels, 2 markers each)
    __analog_channels = ('a_ch1', 'a_ch2', 'a_ch3', 'a_ch4')
    __digital_channels = ('d_ch1', 'd_ch2', 'd_ch3', 'd_ch4',
                          'd_ch5', 'd_ch6', 'd_ch7', 'd_ch8')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._rm = None          # pyvisa ResourceManager (created on activation)
        self.awg = None          # pyvisa resource
        self.awg_model = ''      # from *IDN?
        self.ftp_working_dir = 'waves'  # subfolder of the FTP root on the AWG disk
        self._loaded_assets = {}        # {int channel index: asset name} (internal tracking)
        self._written_wfm_names = set()  # local record of waveforms written this session
        self._wfm_open_handles = {}      # {wfm_name: open file handle} during chunked writes

    # =========================================================================
    # Activation / deactivation
    # =========================================================================

    def on_activate(self):
        """ Connect via VISA/GPIB and verify the instrument identity.

        Deliberately PASSIVE (project safety discipline): no output enable, no run/stop,
        no settings change — except sample rate IF 'default_sample_rate' is explicitly
        configured by the owner.
        """
        # local work directory for generated .wfm files
        if not os.path.exists(self._tmp_work_dir):
            os.makedirs(os.path.abspath(self._tmp_work_dir))

        self._rm = visa.ResourceManager()
        if self._visa_address not in self._rm.list_resources():
            self.awg = None
            raise RuntimeError(
                'VISA address "{0}" not found by the pyVISA resource manager. Check the GPIB '
                'connection (NI MAX) and the address (REQ-028 / AWG front panel).'
                ''.format(self._visa_address))
        self.awg = self._rm.open_resource(self._visa_address)
        self.awg.timeout = int(self._visa_timeout * 1000)  # pyvisa timeout is in ms

        idn = self.query('*IDN?')
        if 'AWG5014' not in idn.replace(' ', ''):
            self.log.warning('Unexpected *IDN? response (no "AWG5014"): "{0}". Wrong GPIB '
                             'address? Proceeding, but check REQ-028.'.format(idn))
        parts = idn.split(',')
        self.awg_model = parts[1].strip() if len(parts) > 1 else idn
        self.log.info('Connected to: {0}'.format(idn))

        if self._ftp_ip_address is None:
            self.log.warning('No "ftp_ip_address" configured: waveform UPLOAD IS DISABLED '
                             '(phase B — the AWG ethernet link is not set up; phase C item).')

        if self._default_sample_rate is not None:
            self.set_sample_rate(float(self._default_sample_rate))

    def on_deactivate(self):
        """ Close the VISA connection. """
        self._close_open_wfm_handles(discard=True)
        try:
            self.awg.close()
        except Exception:
            self.log.debug('Closing AWG VISA connection failed.')
        self.awg = None

    # =========================================================================
    # PulserInterface: constraints
    # =========================================================================

    def get_constraints(self):
        """ Return the PulserConstraints for the AWG5014C.

        PROVISIONAL (REQ-028 open): 5014C-specific limits come from the ConfigOptions
        documented in the class docstring; donor-family values are marked TBC.
        """
        constraints = PulserConstraints()

        constraints.sample_rate.min = float(self._cfg_sample_rate_min)   # TBC(REQ-028)
        constraints.sample_rate.max = float(self._cfg_sample_rate_max)   # TBC(REQ-028)
        constraints.sample_rate.step = 1.0e4
        constraints.sample_rate.default = float(self._cfg_sample_rate_max)

        # donor (AWG5000 family) analog levels — TBC(REQ-028)
        constraints.a_ch_amplitude.min = 0.02
        constraints.a_ch_amplitude.max = 4.5
        constraints.a_ch_amplitude.step = 0.001
        constraints.a_ch_amplitude.default = 4.5

        constraints.a_ch_offset.min = -2.25
        constraints.a_ch_offset.max = 2.25
        constraints.a_ch_offset.step = 0.001
        constraints.a_ch_offset.default = 0.0

        # donor (AWG5000 family) marker levels — TBC(REQ-028)
        constraints.d_ch_low.min = -1.0
        constraints.d_ch_low.max = 2.6
        constraints.d_ch_low.step = 0.01
        constraints.d_ch_low.default = 0.0

        constraints.d_ch_high.min = -0.9
        constraints.d_ch_high.max = 2.7
        constraints.d_ch_high.step = 0.01
        constraints.d_ch_high.default = 2.7

        constraints.waveform_length.min = int(self._cfg_min_wfm_length)   # TBC(REQ-028)
        constraints.waveform_length.max = int(self._cfg_max_wfm_length)   # TBC(REQ-028)
        constraints.waveform_length.step = int(self._cfg_wfm_granularity)  # TBC(REQ-028)
        constraints.waveform_length.default = int(self._cfg_min_wfm_length)

        constraints.waveform_num.min = 1
        constraints.waveform_num.max = 32000        # TBC(REQ-028)
        constraints.waveform_num.step = 1
        constraints.waveform_num.default = 1

        # Sequence mode is a NON-GOAL in phase B: advertise NO sequence capability so the
        # sequence generator logic never routes sequences here. Phase D+ will lift this
        # together with the sourced sequence-table limits from device_models.md.
        constraints.sequence_option = SequenceOption.NON

        # trigger inputs (donor: 'A'/'B'; manual lists external trigger + event input)
        constraints.event_triggers = ['A', 'B']     # TBC(REQ-028)
        constraints.flags = list()

        activation_config = OrderedDict()
        # all four channels with all markers
        activation_config['all'] = frozenset(
            {'a_ch1', 'd_ch1', 'd_ch2', 'a_ch2', 'd_ch3', 'd_ch4',
             'a_ch3', 'd_ch5', 'd_ch6', 'a_ch4', 'd_ch7', 'd_ch8'})
        # channel pair 1+2 with their markers (the IQ + gate working set for CPMG/XY8)
        activation_config['ch12_mk'] = frozenset(
            {'a_ch1', 'd_ch1', 'd_ch2', 'a_ch2', 'd_ch3', 'd_ch4'})
        # single channels with markers
        activation_config['ch1_mk'] = frozenset({'a_ch1', 'd_ch1', 'd_ch2'})
        activation_config['ch2_mk'] = frozenset({'a_ch2', 'd_ch3', 'd_ch4'})
        # analog-only configs (e.g. plain sin/cos two-channel test)
        activation_config['ch12_analog'] = frozenset({'a_ch1', 'a_ch2'})
        activation_config['ch1234_analog'] = frozenset({'a_ch1', 'a_ch2', 'a_ch3', 'a_ch4'})
        constraints.activation_config = activation_config

        return constraints

    # =========================================================================
    # PulserInterface: run control / status
    # =========================================================================

    def pulser_on(self):
        """ Start waveform output.  @return int: qudi status (see get_status) """
        self.write('AWGC:RUN')
        return self.get_status()[0]

    def pulser_off(self):
        """ Stop waveform output.  @return int: qudi status (see get_status) """
        self.write('AWGC:STOP')
        return self.get_status()[0]

    def get_status(self):
        """ Query the run state.

        Manual (077006105 Rev A p. 2-39): AWGControl:RSTate? returns
            0 = stopped, 1 = waiting for trigger, 2 = running.
        Qudi convention: 0 = stopped, 1 = running, 2 = waiting for trigger => remap 1<->2.

        @return (int, dict): current status and description dict (-1 = comms failure)
        """
        status_dic = {-1: 'Failed request or failed communication with device.',
                      0: 'Device has stopped, but can receive commands.',
                      1: 'Device is active and running.',
                      2: 'Device is active and waiting for trigger.'}
        try:
            device_state = int(self.query('AWGC:RSTATE?'))
        except Exception:
            return -1, status_dic
        remap = {0: 0, 1: 2, 2: 1}
        return remap.get(device_state, -1), status_dic

    # =========================================================================
    # PulserInterface: sample rate
    # =========================================================================

    def get_sample_rate(self):
        """ @return float: current sample rate directly from the device (Hz) """
        return float(self.query('SOUR1:FREQ?'))

    def set_sample_rate(self, sample_rate):
        """ Set the sample rate (Hz); returns the value read back from the device. """
        constraints = self.get_constraints()
        sample_rate = float(sample_rate)
        if not (constraints.sample_rate.min <= sample_rate <= constraints.sample_rate.max):
            self.log.error('Requested sample rate {0:.4g} Hz is outside the (provisional, '
                           'REQ-028) allowed range [{1:.4g}, {2:.4g}] Hz. Command ignored.'
                           ''.format(sample_rate, constraints.sample_rate.min,
                                     constraints.sample_rate.max))
            return self.get_sample_rate()
        self.write('SOUR1:FREQ {0:.10e}'.format(sample_rate))
        self.query('*OPC?')
        return self.get_sample_rate()

    # =========================================================================
    # PulserInterface: analog / digital levels
    # =========================================================================

    def get_analog_level(self, amplitude=None, offset=None):
        """ Amplitude (Vpp) and offset (V) of the analog channels; see interface docstring. """
        amp_query = list(self.__analog_channels) if not amplitude else list(amplitude)
        off_query = list(self.__analog_channels) if not offset else list(offset)
        amp = {}
        off = {}
        for a_ch in amp_query:
            ch_num = self._analog_ch_num(a_ch)
            amp[a_ch] = float(self.query('SOUR{0:d}:VOLT:AMPL?'.format(ch_num)))
        for a_ch in off_query:
            ch_num = self._analog_ch_num(a_ch)
            off[a_ch] = float(self.query('SOUR{0:d}:VOLT:OFFS?'.format(ch_num)))
        return amp, off

    def set_analog_level(self, amplitude=None, offset=None):
        """ Set amplitude (Vpp) / offset (V) per analog channel; see interface docstring. """
        if amplitude is None:
            amplitude = {}
        if offset is None:
            offset = {}
        constraints = self.get_constraints()

        for a_ch, value in amplitude.items():
            ch_num = self._analog_ch_num(a_ch)
            constr = constraints.a_ch_amplitude
            if not (constr.min <= value <= constr.max):
                self.log.warning('Amplitude {0} Vpp for {1} outside [{2}, {3}] Vpp '
                                 '(provisional, REQ-028). Command ignored.'
                                 ''.format(value, a_ch, constr.min, constr.max))
                continue
            self.write('SOUR{0:d}:VOLT:AMPL {1}'.format(ch_num, value))
        for a_ch, value in offset.items():
            ch_num = self._analog_ch_num(a_ch)
            constr = constraints.a_ch_offset
            if not (constr.min <= value <= constr.max):
                self.log.warning('Offset {0} V for {1} outside [{2}, {3}] V (provisional, '
                                 'REQ-028). Command ignored.'
                                 ''.format(value, a_ch, constr.min, constr.max))
                continue
            self.write('SOUR{0:d}:VOLT:OFFS {1}'.format(ch_num, value))
        return self.get_analog_level(amplitude=list(amplitude), offset=list(offset))

    def get_digital_level(self, low=None, high=None):
        """ Marker low/high levels (V); d_ch<k> maps to SOURce<(k+1)//2>:MARKer<2-(k%2)>. """
        low_query = list(self.__digital_channels) if not low else list(low)
        high_query = list(self.__digital_channels) if not high else list(high)
        low_val = {}
        high_val = {}
        for d_ch in low_query:
            src, mrk = self._marker_of(d_ch)
            low_val[d_ch] = float(self.query('SOUR{0:d}:MARK{1:d}:VOLT:LOW?'.format(src, mrk)))
        for d_ch in high_query:
            src, mrk = self._marker_of(d_ch)
            high_val[d_ch] = float(self.query('SOUR{0:d}:MARK{1:d}:VOLT:HIGH?'.format(src, mrk)))
        return low_val, high_val

    def set_digital_level(self, low=None, high=None):
        """ Set marker low/high levels (V); see get_digital_level for the mapping. """
        if low is None:
            low = {}
        if high is None:
            high = {}
        constraints = self.get_constraints()

        for d_ch, value in low.items():
            src, mrk = self._marker_of(d_ch)
            constr = constraints.d_ch_low
            if not (constr.min <= value <= constr.max):
                self.log.warning('Marker low {0} V for {1} outside [{2}, {3}] V (provisional, '
                                 'REQ-028). Command ignored.'
                                 ''.format(value, d_ch, constr.min, constr.max))
                continue
            self.write('SOUR{0:d}:MARK{1:d}:VOLT:LOW {2}'.format(src, mrk, value))
        for d_ch, value in high.items():
            src, mrk = self._marker_of(d_ch)
            constr = constraints.d_ch_high
            if not (constr.min <= value <= constr.max):
                self.log.warning('Marker high {0} V for {1} outside [{2}, {3}] V (provisional, '
                                 'REQ-028). Command ignored.'
                                 ''.format(value, d_ch, constr.min, constr.max))
                continue
            self.write('SOUR{0:d}:MARK{1:d}:VOLT:HIGH {2}'.format(src, mrk, value))
        return self.get_digital_level(low=list(low), high=list(high))

    # =========================================================================
    # PulserInterface: channel activation
    # =========================================================================

    def get_active_channels(self, ch=None):
        """ Active channels. Analog: OUTPut<n>:STATe?. Markers: the AWG5000 series runs at
        fixed 14-bit DAC resolution, so markers are always available — a marker is reported
        active iff its parent analog channel output is on (donor behavior refined). """
        if ch is None:
            ch = list(self.__analog_channels) + list(self.__digital_channels)
        active_ch = {}
        analog_state_cache = {}
        for channel in ch:
            if channel in self.__analog_channels:
                ch_num = self._analog_ch_num(channel)
                if ch_num not in analog_state_cache:
                    analog_state_cache[ch_num] = bool(
                        int(self.query('OUTP{0:d}:STAT?'.format(ch_num))))
                active_ch[channel] = analog_state_cache[ch_num]
            elif channel in self.__digital_channels:
                src, _ = self._marker_of(channel)
                if src not in analog_state_cache:
                    analog_state_cache[src] = bool(
                        int(self.query('OUTP{0:d}:STAT?'.format(src))))
                active_ch[channel] = analog_state_cache[src]
            else:
                raise ValueError('Unknown channel descriptor "{0}" for AWG5014C (valid: '
                                 'a_ch1..a_ch4, d_ch1..d_ch8).'.format(channel))
        return active_ch

    def set_active_channels(self, ch=None):
        """ Set analog outputs on/off (OUTPut<n>:STATe). Marker entries are accepted but the
        AWG5000 series cannot deactivate markers independently (fixed 14-bit DAC) — they
        follow their parent analog channel; mismatching marker requests are logged. """
        if ch is None:
            ch = {}
        for channel, state in ch.items():
            if channel in self.__analog_channels:
                ch_num = self._analog_ch_num(channel)
                self.write('OUTP{0:d}:STAT {1}'.format(ch_num, 'ON' if state else 'OFF'))
            elif channel in self.__digital_channels:
                self.log.debug('Marker channel {0} activation follows its parent analog '
                               'channel on the AWG5000 series; entry ignored.'.format(channel))
            else:
                raise ValueError('Unknown channel descriptor "{0}" for AWG5014C (valid: '
                                 'a_ch1..a_ch4, d_ch1..d_ch8).'.format(channel))
        return self.get_active_channels()

    # =========================================================================
    # PulserInterface: waveform write / load / bookkeeping
    # =========================================================================

    def write_waveform(self, name, analog_samples, digital_samples, is_first_chunk,
                       is_last_chunk, total_number_of_samples):
        """ Write .wfm files (one per active analog channel) locally and upload them via FTP.

        ERROR CONTRACT (phase-B validated): illegal total length (below min, above max, or
        violating the granularity) and unknown channel descriptors RAISE ValueError before
        any byte is written — waveforms are never silently truncated or padded.

        @return (int, list): samples written (-1 on failure) and created waveform names
        """
        waveforms = list()

        # ---- validate channel descriptors (raise: contract) ----
        unknown = [k for k in analog_samples if k not in self.__analog_channels]
        unknown += [k for k in digital_samples if k not in self.__digital_channels]
        if unknown:
            self.log.error('write_waveform: unknown channel descriptor(s) {0} for AWG5014C.'
                           ''.format(unknown))
            raise ValueError('Unknown channel descriptor(s) for AWG5014C: {0} (valid: '
                             'a_ch1..a_ch4, d_ch1..d_ch8).'.format(unknown))

        if len(analog_samples) == 0:
            self.log.error('write_waveform: no analog samples passed (AWG5014C waveforms are '
                           'written per analog channel).')
            return -1, waveforms

        # ---- validate total length ONCE, before any file is touched (raise: contract) ----
        if is_first_chunk:
            constraints = self.get_constraints()
            total = int(total_number_of_samples)
            if total < constraints.waveform_length.min:
                msg = ('Waveform length {0:d} is below the minimum of {1:d} samples '
                       '(provisional constraint, REQ-028).'
                       ''.format(total, constraints.waveform_length.min))
                self.log.error('write_waveform: ' + msg)
                raise ValueError(msg)
            if total > constraints.waveform_length.max:
                msg = ('Waveform length {0:d} exceeds the maximum of {1:d} samples '
                       '(provisional constraint, REQ-028).'
                       ''.format(total, constraints.waveform_length.max))
                self.log.error('write_waveform: ' + msg)
                raise ValueError(msg)
            granularity = constraints.waveform_length.step
            if total % granularity != 0:
                msg = ('Waveform length {0:d} violates the length granularity {1:d} '
                       '(provisional constraint, REQ-028). Refusing to truncate/pad.'
                       ''.format(total, granularity))
                self.log.error('write_waveform: ' + msg)
                raise ValueError(msg)

        # ---- validate chunk consistency ----
        chunk_length = len(analog_samples[list(analog_samples)[0]])
        for chnl, samples in list(analog_samples.items()) + list(digital_samples.items()):
            if len(samples) != chunk_length:
                self.log.error('write_waveform: unequal sample array lengths across channels.')
                return -1, waveforms

        # ---- congruence with channel activation (awg70k pattern, analog channels) ----
        activation = self.get_active_channels()
        active_analog = natural_sort(c for c in self.__analog_channels if activation[c])
        if set(analog_samples) != set(active_analog):
            self.log.error('write_waveform: mismatch between active analog channels {0} and '
                           'provided sample arrays {1}.'
                           ''.format(active_analog, sorted(analog_samples)))
            return -1, waveforms

        # ---- write one .wfm per analog channel ----
        sample_rate = self.get_sample_rate()
        for a_ch in active_analog:
            a_ch_num = self._analog_ch_num(a_ch)
            wfm_name = '{0}_ch{1:d}'.format(name, a_ch_num)

            mrk1 = 'd_ch{0:d}'.format(2 * a_ch_num - 1)
            mrk2 = 'd_ch{0:d}'.format(2 * a_ch_num)
            # marker byte: bit 6 = marker 1, bit 7 = marker 2 (family layout, TBC REQ-028)
            marker_bytes = np.zeros(chunk_length, dtype=np.uint8)
            if mrk1 in digital_samples:
                np.add(marker_bytes,
                       np.left_shift(digital_samples[mrk1].astype(np.uint8), 6),
                       out=marker_bytes)
            if mrk2 in digital_samples:
                np.add(marker_bytes,
                       np.left_shift(digital_samples[mrk2].astype(np.uint8), 7),
                       out=marker_bytes)

            self._write_wfm_chunk(wfm_name=wfm_name,
                                  analog_chunk=analog_samples[a_ch],
                                  marker_bytes=marker_bytes,
                                  is_first_chunk=is_first_chunk,
                                  is_last_chunk=is_last_chunk,
                                  total_number_of_samples=int(total_number_of_samples),
                                  sample_rate=sample_rate)

            if is_last_chunk:
                self._send_file(wfm_name + '.wfm')
                self._written_wfm_names.add(wfm_name)
            waveforms.append(wfm_name)

        # sequence_generator_logic compares the return value against the number of samples
        # staged in THIS call (chunk), not the ensemble total -> return the chunk length.
        return chunk_length, waveforms

    def write_sequence(self, name, sequence_parameters):
        """ NOT AVAILABLE in phase B (non-goal; sequence_option = NON).  @return int: -1 """
        self.log.error('Sequence mode is not implemented in the phase-B AWG5014C module '
                       '(non-goal; planned phase D+, gated on REQ-028 sequence-table limits).')
        return -1

    def get_waveform_names(self):
        """ Waveform files (*.wfm, stripped extension) in the AWG's FTP waveform directory.
        Without a configured FTP address (phase B) falls back to the local session record. """
        if self._ftp_ip_address is None:
            return natural_sort(self._written_wfm_names)
        names = []
        for filename in self._get_filenames_on_device():
            if filename.endswith('.wfm'):
                names.append(filename[:-4])
        return natural_sort(set(names))

    def get_sequence_names(self):
        """ Sequence mode is a phase-B non-goal.  @return list: empty """
        return list()

    def delete_waveform(self, waveform_name):
        """ Delete waveform file(s) from the AWG's FTP waveform directory.
        @return list: deleted waveform names """
        if isinstance(waveform_name, str):
            waveform_name = [waveform_name]
        avail = self.get_waveform_names()
        to_delete = [wfm for wfm in waveform_name if wfm in avail]
        if self._ftp_ip_address is None:
            self._written_wfm_names.difference_update(to_delete)
            return to_delete
        with FTP(self._ftp_ip_address) as ftp:
            ftp.login(user=self._username, passwd=self._password)
            ftp.cwd(self.ftp_working_dir)
            for wfm in to_delete:
                ftp.delete(wfm + '.wfm')
        self._written_wfm_names.difference_update(to_delete)
        return to_delete

    def delete_sequence(self, sequence_name):
        """ Sequence mode is a phase-B non-goal.  @return list: empty """
        return list()

    def load_waveform(self, load_dict):
        """ Load written .wfm files into channels via [SOURce[n]]:FUNCtion:USER
        (AWG5000-series command, programmer manual 077006105).

        @param dict|list load_dict: {channel index: waveform name} or list of names with
                                    '_ch<n>' suffixes (see interface docstring)
        @return dict: actually loaded waveforms per channel
        """
        if isinstance(load_dict, list):
            new_dict = dict()
            for waveform in load_dict:
                channel = int(waveform.rsplit('_ch', 1)[1])
                new_dict[channel] = waveform
            load_dict = new_dict

        # unknown channel index -> clean raise (error contract)
        invalid = [ch for ch in load_dict if ch not in (1, 2, 3, 4)]
        if invalid:
            self.log.error('load_waveform: invalid channel index(es) {0} (AWG5014C has '
                           'channels 1..4).'.format(invalid))
            raise ValueError('Invalid AWG5014C channel index(es): {0} (valid: 1..4).'
                             ''.format(invalid))

        # all waveforms must exist on the device (or local record in phase B)
        avail = self.get_waveform_names()
        missing = [wfm for wfm in load_dict.values() if wfm not in avail]
        if missing:
            self.log.error('load_waveform: waveform(s) {0} not found on the device.'
                           ''.format(missing))
            return self.get_loaded_assets()[0]

        path = self._ftp_root_dir + '\\' + self.ftp_working_dir
        for ch_num, wfm in load_dict.items():
            self.write('SOUR{0:d}:FUNC:USER "{1}/{2}"'.format(ch_num, path, wfm + '.wfm'))
            self._loaded_assets[ch_num] = wfm
        self.query('*OPC?')
        return self.get_loaded_assets()[0]

    def load_sequence(self, sequence_name):
        """ NOT AVAILABLE in phase B (non-goal).  @return dict: currently loaded assets """
        self.log.error('Sequence mode is not implemented in the phase-B AWG5014C module '
                       '(non-goal; planned phase D+).')
        return self.get_loaded_assets()[0]

    def get_loaded_assets(self):
        """ Internally tracked channel->asset map (phase B; cross-check against
        SOUR<n>:FUNC:USER? readback is a phase-C hardware-validation item).

        @return (dict, str): {channel index: asset name}, 'waveform' (sequences: phase D+)
        """
        return dict(self._loaded_assets), 'waveform'

    def clear_all(self):
        """ Delete all waveforms from the AWG waveform list and forget loaded assets.
        @return int: error code (0: OK) """
        self.write('WLIS:WAV:DEL ALL')
        self._loaded_assets = {}
        return 0

    # =========================================================================
    # PulserInterface: misc
    # =========================================================================

    def get_interleave(self):
        """ The AWG5000 series has no interleave; always False. """
        return False

    def set_interleave(self, state=False):
        """ No interleave on the AWG5000 series; warns if activation is requested. """
        if state:
            self.log.warning('Interleave mode not available for the AWG5000 series. '
                             'Method call ignored.')
        return False

    def reset(self):
        """ *RST and clear the internal asset bookkeeping.  @return int: 0 """
        self.write('*RST')
        self.query('*OPC?')
        self._loaded_assets = {}
        self._close_open_wfm_handles(discard=True)
        return 0

    # =========================================================================
    # Communication helpers (VISA/GPIB)
    # =========================================================================

    def query(self, question):
        """ Query the device and return the stripped answer string. """
        return self.awg.query(question).strip().strip('"')

    def write(self, command):
        """ Write a command to the device.  @return int: 0 """
        self.awg.write(command)
        return 0

    # =========================================================================
    # Internal helpers
    # =========================================================================

    def _analog_ch_num(self, a_ch):
        """ 'a_ch3' -> 3, validating the descriptor (raise ValueError otherwise). """
        if a_ch not in self.__analog_channels:
            raise ValueError('Unknown analog channel descriptor "{0}" for AWG5014C (valid: '
                             'a_ch1..a_ch4).'.format(a_ch))
        return int(a_ch.rsplit('ch', 1)[1])

    def _marker_of(self, d_ch):
        """ 'd_ch<k>' -> (source channel 1..4, marker index 1..2); raises on invalid. """
        if d_ch not in self.__digital_channels:
            raise ValueError('Unknown digital channel descriptor "{0}" for AWG5014C (valid: '
                             'd_ch1..d_ch8).'.format(d_ch))
        k = int(d_ch.rsplit('ch', 1)[1])
        return (k + 1) // 2, 2 - (k % 2)

    def _write_wfm_chunk(self, wfm_name, analog_chunk, marker_bytes, is_first_chunk,
                         is_last_chunk, total_number_of_samples, sample_rate):
        """ Append one chunk to <tmp_work_dir>/<wfm_name>.wfm in the family-standard layout:

            b'MAGIC 1000\\r\\n'
            b'#' + <ndigits> + <nbytes>          (nbytes = 5 * total_number_of_samples)
            total_number_of_samples * [ <float32-LE analog in [-1, 1]> + <1 marker byte> ]
            b'CLOCK <rate as %16.10E>\\r\\n'

        Byte layout TBC(REQ-028) against the full programmer manual before phase-C upload.
        """
        filepath = os.path.join(self._tmp_work_dir, wfm_name + '.wfm')
        if is_first_chunk:
            wfm_file = open(filepath, 'wb')
            self._wfm_open_handles[wfm_name] = wfm_file
            num_bytes = str(int(total_number_of_samples * 5))
            header = b'MAGIC 1000\r\n' + b'#' + str(len(num_bytes)).encode() \
                     + num_bytes.encode()
            wfm_file.write(header)
        else:
            wfm_file = self._wfm_open_handles[wfm_name]

        # interleave float32-LE analog samples with the marker bytes (5 bytes per sample)
        interleaved = np.zeros(len(analog_chunk),
                               dtype=np.dtype([('a', '<f4'), ('m', 'u1')]))
        interleaved['a'] = np.asarray(analog_chunk, dtype='<f4')
        interleaved['m'] = marker_bytes
        wfm_file.write(interleaved.tobytes())

        if is_last_chunk:
            wfm_file.write('CLOCK {0:16.10E}\r\n'.format(sample_rate).encode())
            wfm_file.close()
            del self._wfm_open_handles[wfm_name]

    def _close_open_wfm_handles(self, discard=False):
        """ Close (and optionally delete) any half-written .wfm files. """
        for wfm_name, handle in list(self._wfm_open_handles.items()):
            try:
                handle.close()
            except Exception:
                pass
            if discard:
                try:
                    os.remove(os.path.join(self._tmp_work_dir, wfm_name + '.wfm'))
                except OSError:
                    pass
            del self._wfm_open_handles[wfm_name]

    def _send_file(self, filename):
        """ Upload a file from tmp_work_dir to the AWG waveform directory via FTP (donor
        path). Without a configured FTP address (phase B) this raises RuntimeError. """
        if self._ftp_ip_address is None:
            raise RuntimeError(
                'FTP upload requested but no "ftp_ip_address" is configured. The AWG5014C '
                'ethernet link is not set up (phase C, connections.yaml to_be_confirmed) — '
                'refusing to pretend the upload happened.')
        filepath = os.path.join(self._tmp_work_dir, filename)
        with FTP(self._ftp_ip_address) as ftp:
            ftp.login(user=self._username, passwd=self._password)
            ftp.cwd(self.ftp_working_dir)
            with open(filepath, 'rb') as file_handle:
                ftp.storbinary('STOR ' + filename, file_handle)

    def _get_filenames_on_device(self):
        """ File names in the AWG's FTP waveform directory (donor's listing parser). """
        filename_list = []
        with FTP(self._ftp_ip_address) as ftp:
            ftp.login(user=self._username, passwd=self._password)
            ftp.cwd(self.ftp_working_dir)
            log_lines = []
            ftp.retrlines('LIST', callback=log_lines.append)
            for line in log_lines:
                if '<DIR>' in line:
                    continue
                # e.g. '05-10-16  05:22PM        292 SSR aom adjusted.seq' (donor comment)
                size_and_name = line[18:].lstrip()
                actual_filename = size_and_name.split(' ', 1)[1].lstrip()
                if fnmatch(actual_filename, '*.wfm') or fnmatch(actual_filename, '*.seq'):
                    if actual_filename not in filename_list:
                        filename_list.append(actual_filename)
        return filename_list
