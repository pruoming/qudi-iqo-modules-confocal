# -*- coding: utf-8 -*-

"""
Qudi hardware module for the Tektronix AWG5014C arbitrary waveform generator.

FORK MODULE (confocal_odmr, cycle 2, AWG intake — phase B 2026-07-22, revised same day
after REQ-028 was answered). Originally adapted from the legacy-interface donor
tektronix_awg5002c.py and ported to the CURRENT PulserInterface following the
tektronix_awg70k.py patterns (pyvisa transport, write_waveform/load_waveform API).

SCOPE (phase B+, mock-first; no instrument contact by this module yet):
  * SCPI control via VISA/GPIB (NI GPIB-USB-HS; GPIB0::1::INSTR probe-confirmed
    2026-07-22, connections.yaml devices.awg).
  * Waveform TRANSFER via SCPI WLISt:WAVeform:DATA over the SAME VISA link (works over
    GPIB — no ethernet needed). The donor's FTP/.wfm path was REMOVED 2026-07-22
    (REQ-030 decision, operator-approved): the .wfm file byte layout was never sourced,
    while the WLISt integer format IS sourced (see below).
  * SEQUENCE MODE IS OUT OF SCOPE (non-goal this phase): sequence_option = NON,
    write/load_sequence fail loudly. Planned for phase D+.

CONSTRAINTS RECONCILED 2026-07-22 (REQ-028 answered — shared_knowledge/device_models.md
AWG5014C entry, sourced from Tektronix Technical Reference 077-0455-03 spec tables;
in-repo extract in setups/confocal_odmr/hardware_manuals/). The limits remain
ConfigOptions so another setup/unit can override them, but the DEFAULTS are now the
sourced values:
    sample_rate:          10 MS/s .. 1.2 GS/s          (Tech Ref Table 3)
    min_waveform_length:  250 points (HARDWARE limit)  (Tech Ref Table 2)
    waveform_granularity: 1 point                      (Tech Ref Table 2)
    max_waveform_length:  32_400_000 (Opt 01 — installed on this unit, REQ-028;
                          base instrument without Opt 01: 16_200_000)
    analog levels:        20 mV..4.5 Vp-p SE Normal, offset ±2.25 V (Table 7)
    marker levels:        window −1.0..+2.7 V, min amplitude 0.1 V (Table 8)

SOURCED PROTOCOL FACTS:
  * Integer waveform data format (Programmer Manual 077-0061-05 Table 2-25, in-repo
    extract): 2 bytes per point, transferred LSB-first; bit layout M2 M1 D13..D0
    (14 data bits, marker 1 = bit 14, marker 2 = bit 15).
  * WLISt:WAVeform:DATA supports chunked transfer via StartIndex/Size parameters
    (extract, "Transferring Waveforms in Chunks") — NOT used here: the exact chunk
    argument order is on a manual page missing from the extract, so this module
    accumulates qudi's chunks locally and sends ONE binary block per waveform.
    Upgrade to true chunked SCPI transfer only with the full programmer manual.
  * Exact command strings corroborated 2026-07-22 against the QCoDeS production driver
    for this instrument (working code against real 5014C hardware; see
    https://github.com/microsoft/Qcodes src/qcodes/instrument_drivers/tektronix/AWG5014.py):
        WLISt:WAVeform:NEW "<name>",<size>,INTEGER
        WLISt:WAVeform:DATA "<name>",#<ndigits><nbytes><binary LSB-first>
        WLISt:WAVeform:DEL "<name>"   /   WLISt:WAVeform:DELete ALL
        SOURce<n>:WAVeform "<name>"        (load from the waveform list into channel n)
    Analog scaling follows the same driver: a in [-1, 1] -> DAC code round(8191 + 8191*a)
    (integer format is the raw hardware word; the manual's REAL format is the normalized
    ±1 representation, Table 2-26).
  * AWGControl:RSTate? returns 0=stopped, 1=waiting-for-trigger, 2=running (p. 2-39);
    qudi status ints are 0=stopped, 1=running, 2=waiting => mapping in get_status().
  * AWGControl:RMODe {CONTinuous|TRIGgered|GATed|SEQuence} (p. 2-37).

PHASE-C VERIFICATION ITEMS (first instrument contact):
  * After the first real transfer, drain SYSTem:ERRor? and verify the waveform appears
    in the instrument's waveform list and outputs the expected shape on a scope.
  * get_waveform_names() uses the LOCAL session record (the WLISt:NAME?/SIZE? query
    syntax page is not in the extract) — switch to the instrument query at phase C.
  * Whether *RST clears the instrument's user waveform list is unverified.

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

from collections import OrderedDict

import numpy as np

try:
    import pyvisa as visa
except ImportError:
    import visa

from qudi.core.configoption import ConfigOption
from qudi.util.helpers import natural_sort
from qudi.interface.pulser_interface import PulserInterface, PulserConstraints, SequenceOption


class AWG5014C(PulserInterface):
    """ Tektronix AWG5014C: 4 analog channels, 2 markers per channel (d_ch1..d_ch8).

    Marker mapping (fixed by hardware topology):
        a_ch<n>  <->  markers d_ch<2n-1> (marker 1) and d_ch<2n> (marker 2)
        i.e. a_ch1: d_ch1/d_ch2, a_ch2: d_ch3/d_ch4, a_ch3: d_ch5/d_ch6, a_ch4: d_ch7/d_ch8

    Example config for copy-paste (waveform transfer runs over the GPIB/VISA link —
    no ethernet required):

    pulser_awg5014c:
        module.Class: 'awg.tektronix_awg5014c.AWG5014C'
        options:
            awg_visa_address: 'GPIB0::1::INSTR'   # probe-confirmed 2026-07-22 (REQ-028)
            timeout: 10                            # VISA timeout in seconds
            # default_sample_rate: 1.2e9   # applied on activation ONLY if set
            # --- constraint overrides; defaults are SOURCED (device_models.md AWG5014C) ---
            # sample_rate_min: 10.0e6
            # sample_rate_max: 1.2e9
            # min_waveform_length: 250
            # waveform_granularity: 1
            # max_waveform_length: 32400000   # Opt 01 value; base unit: 16200000
    """

    # ---- transport config ----
    _visa_address = ConfigOption(name='awg_visa_address', missing='error')
    _visa_timeout = ConfigOption(name='timeout', default=10, missing='warn')  # seconds
    _default_sample_rate = ConfigOption(name='default_sample_rate', default=None,
                                        missing='nothing')

    # ---- device constraints — SOURCED defaults (REQ-028 answered 2026-07-22;
    #      device_models.md AWG5014C / Tech Ref 077-0455-03), overridable per unit ----
    _cfg_sample_rate_min = ConfigOption(name='sample_rate_min', default=10.0e6,
                                        missing='nothing')   # Table 3
    _cfg_sample_rate_max = ConfigOption(name='sample_rate_max', default=1.2e9,
                                        missing='nothing')   # Table 3
    _cfg_min_wfm_length = ConfigOption(name='min_waveform_length', default=250,
                                       missing='nothing')    # Table 2 (HW limit)
    _cfg_wfm_granularity = ConfigOption(name='waveform_granularity', default=1,
                                        missing='nothing')   # Table 2
    _cfg_max_wfm_length = ConfigOption(name='max_waveform_length', default=32_400_000,
                                       missing='nothing')    # Table 2, Opt 01 (this unit)

    # channel topology (structural: 4 analog channels, 2 markers each)
    __analog_channels = ('a_ch1', 'a_ch2', 'a_ch3', 'a_ch4')
    __digital_channels = ('d_ch1', 'd_ch2', 'd_ch3', 'd_ch4',
                          'd_ch5', 'd_ch6', 'd_ch7', 'd_ch8')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._rm = None          # pyvisa ResourceManager (created on activation)
        self.awg = None          # pyvisa resource
        self.awg_model = ''      # from *IDN?
        self._loaded_assets = {}        # {int channel index: asset name} (internal tracking)
        self._written_wfm_names = set()  # local record of waveforms transferred this session
        self._wfm_buffers = {}   # {wfm_name: [bytes, ...]} packed chunks awaiting transfer
        self._wfm_totals = {}    # {wfm_name: expected total number of samples}

    # =========================================================================
    # Activation / deactivation
    # =========================================================================

    def on_activate(self):
        """ Connect via VISA/GPIB and verify the instrument identity.

        Deliberately PASSIVE (project safety discipline): no output enable, no run/stop,
        no settings change — except sample rate IF 'default_sample_rate' is explicitly
        configured by the owner.
        """
        self._rm = visa.ResourceManager()
        if self._visa_address not in self._rm.list_resources():
            self.awg = None
            raise RuntimeError(
                'VISA address "{0}" not found by the pyVISA resource manager. Check the GPIB '
                'connection (NI MAX) and the address (connections.yaml devices.awg).'
                ''.format(self._visa_address))
        self.awg = self._rm.open_resource(self._visa_address)
        self.awg.timeout = int(self._visa_timeout * 1000)  # pyvisa timeout is in ms

        idn = self.query('*IDN?')
        if 'AWG5014' not in idn.replace(' ', ''):
            self.log.warning('Unexpected *IDN? response (no "AWG5014"): "{0}". Wrong GPIB '
                             'address? Proceeding, but check connections.yaml devices.awg.'
                             ''.format(idn))
        parts = idn.split(',')
        self.awg_model = parts[1].strip() if len(parts) > 1 else idn
        self.log.info('Connected to: {0}'.format(idn))

        if self._default_sample_rate is not None:
            self.set_sample_rate(float(self._default_sample_rate))

    def on_deactivate(self):
        """ Close the VISA connection; discard any half-buffered waveform chunks. """
        self._discard_wfm_buffers()
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

        Defaults SOURCED (REQ-028 answered 2026-07-22): device_models.md AWG5014C entry,
        from the Tektronix Technical Reference 077-0455-03 spec tables (in-repo extract).
        The ConfigOptions allow per-unit overrides (e.g. a unit without Opt 01).
        """
        constraints = PulserConstraints()

        constraints.sample_rate.min = float(self._cfg_sample_rate_min)   # Table 3
        constraints.sample_rate.max = float(self._cfg_sample_rate_max)   # Table 3
        constraints.sample_rate.step = 10.0   # Table 3: 8-digit resolution (10 S/s at max)
        constraints.sample_rate.default = float(self._cfg_sample_rate_max)

        # analog levels — Tech Ref Table 7 (per SE output into 50 ohm, Normal path)
        constraints.a_ch_amplitude.min = 0.02
        constraints.a_ch_amplitude.max = 4.5
        constraints.a_ch_amplitude.step = 0.001
        constraints.a_ch_amplitude.default = 4.5

        constraints.a_ch_offset.min = -2.25
        constraints.a_ch_offset.max = 2.25
        constraints.a_ch_offset.step = 0.001
        constraints.a_ch_offset.default = 0.0

        # marker levels — Tech Ref Table 8 (window -1.0..+2.7 V into 50 ohm,
        # amplitude 0.1..3.7 Vp-p, 10 mV resolution)
        constraints.d_ch_low.min = -1.0
        constraints.d_ch_low.max = 2.6
        constraints.d_ch_low.step = 0.01
        constraints.d_ch_low.default = 0.0

        constraints.d_ch_high.min = -0.9
        constraints.d_ch_high.max = 2.7
        constraints.d_ch_high.step = 0.01
        constraints.d_ch_high.default = 2.7

        constraints.waveform_length.min = int(self._cfg_min_wfm_length)   # Table 2 (HW min)
        constraints.waveform_length.max = int(self._cfg_max_wfm_length)   # Table 2 (Opt 01)
        constraints.waveform_length.step = int(self._cfg_wfm_granularity)  # Table 2
        constraints.waveform_length.default = int(self._cfg_min_wfm_length)

        # Tek docs disagree on the waveform-count limit (32,000 Tech Ref vs 16,200
        # datasheet) — use the SAFE bound (device_models.md known_quirks).
        constraints.waveform_num.min = 1
        constraints.waveform_num.max = 16_200
        constraints.waveform_num.step = 1
        constraints.waveform_num.default = 1

        # Sequence mode is a NON-GOAL in this phase: advertise NO sequence capability so
        # the sequence generator logic never routes sequences here. Phase D+ lifts this
        # together with the sourced sequence-table limits from device_models.md.
        constraints.sequence_option = SequenceOption.NON

        # The 5014C has ONE external Trigger In + ONE Event In (Tech Ref Tables 10/11) —
        # no named 'A'/'B' triggers. Unused while sequence_option = NON; revisit phase D.
        constraints.event_triggers = list()
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
            self.log.error('Requested sample rate {0:.4g} Hz is outside the allowed range '
                           '[{1:.4g}, {2:.4g}] Hz (device_models.md AWG5014C). Command '
                           'ignored.'.format(sample_rate, constraints.sample_rate.min,
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
                                 '(Tech Ref Table 7). Command ignored.'
                                 ''.format(value, a_ch, constr.min, constr.max))
                continue
            self.write('SOUR{0:d}:VOLT:AMPL {1}'.format(ch_num, value))
        for a_ch, value in offset.items():
            ch_num = self._analog_ch_num(a_ch)
            constr = constraints.a_ch_offset
            if not (constr.min <= value <= constr.max):
                self.log.warning('Offset {0} V for {1} outside [{2}, {3}] V (Tech Ref '
                                 'Table 7). Command ignored.'
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
                self.log.warning('Marker low {0} V for {1} outside [{2}, {3}] V (Tech Ref '
                                 'Table 8). Command ignored.'
                                 ''.format(value, d_ch, constr.min, constr.max))
                continue
            self.write('SOUR{0:d}:MARK{1:d}:VOLT:LOW {2}'.format(src, mrk, value))
        for d_ch, value in high.items():
            src, mrk = self._marker_of(d_ch)
            constr = constraints.d_ch_high
            if not (constr.min <= value <= constr.max):
                self.log.warning('Marker high {0} V for {1} outside [{2}, {3}] V (Tech Ref '
                                 'Table 8). Command ignored.'
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
        """ Pack samples into the sourced WLISt integer format (Table 2-25) and transfer
        one waveform per active analog channel via WLISt:WAVeform:NEW/DATA.

        Qudi delivers samples in chunks; the packed words are BUFFERED locally and sent
        to the instrument as ONE binary block on the last chunk (see module docstring —
        avoids the unsourced chunked-DATA argument order). Nothing reaches the
        instrument before the last chunk.

        ERROR CONTRACT (shakeout-validated): illegal total length (below min, above max,
        or violating the granularity) and unknown channel descriptors RAISE ValueError
        before any byte is buffered — waveforms are never silently truncated or padded.

        @return (int, list): samples written this chunk (-1 on failure) and waveform names
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

        # ---- validate total length ONCE, before anything is buffered (raise: contract) ----
        if is_first_chunk:
            constraints = self.get_constraints()
            total = int(total_number_of_samples)
            if total < constraints.waveform_length.min:
                msg = ('Waveform length {0:d} is below the hardware minimum of {1:d} samples '
                       '(Tech Ref Table 2).'.format(total, constraints.waveform_length.min))
                self.log.error('write_waveform: ' + msg)
                raise ValueError(msg)
            if total > constraints.waveform_length.max:
                msg = ('Waveform length {0:d} exceeds the maximum of {1:d} samples '
                       '(Tech Ref Table 2, Opt 01).'
                       ''.format(total, constraints.waveform_length.max))
                self.log.error('write_waveform: ' + msg)
                raise ValueError(msg)
            granularity = constraints.waveform_length.step
            if total % granularity != 0:
                msg = ('Waveform length {0:d} violates the length granularity {1:d}. '
                       'Refusing to truncate/pad.'.format(total, granularity))
                self.log.error('write_waveform: ' + msg)
                raise ValueError(msg)

        # ---- validate chunk consistency ----
        chunk_length = len(analog_samples[list(analog_samples)[0]])
        for chnl, samples in list(analog_samples.items()) + list(digital_samples.items()):
            if len(samples) != chunk_length:
                self.log.error('write_waveform: unequal sample array lengths across channels.')
                self._discard_wfm_buffers(prefix=name)
                return -1, waveforms

        # ---- congruence with channel activation (awg70k pattern, analog channels) ----
        activation = self.get_active_channels()
        active_analog = natural_sort(c for c in self.__analog_channels if activation[c])
        if set(analog_samples) != set(active_analog):
            self.log.error('write_waveform: mismatch between active analog channels {0} and '
                           'provided sample arrays {1}.'
                           ''.format(active_analog, sorted(analog_samples)))
            self._discard_wfm_buffers(prefix=name)
            return -1, waveforms

        # ---- pack + buffer this chunk; transfer on the last chunk ----
        for a_ch in active_analog:
            a_ch_num = self._analog_ch_num(a_ch)
            wfm_name = '{0}_ch{1:d}'.format(name, a_ch_num)

            mrk1 = 'd_ch{0:d}'.format(2 * a_ch_num - 1)   # marker 1 = bit 14 (Table 2-25)
            mrk2 = 'd_ch{0:d}'.format(2 * a_ch_num)       # marker 2 = bit 15 (Table 2-25)
            words = self._pack_samples(analog_samples[a_ch],
                                       digital_samples.get(mrk1),
                                       digital_samples.get(mrk2))
            if is_first_chunk:
                self._wfm_buffers[wfm_name] = []
                self._wfm_totals[wfm_name] = int(total_number_of_samples)
            self._wfm_buffers[wfm_name].append(words.tobytes())

            if is_last_chunk:
                self._transfer_waveform(wfm_name)
                self._written_wfm_names.add(wfm_name)
            waveforms.append(wfm_name)

        # sequence_generator_logic compares the return value against the number of samples
        # staged in THIS call (chunk), not the ensemble total -> return the chunk length.
        return chunk_length, waveforms

    def write_sequence(self, name, sequence_parameters):
        """ NOT AVAILABLE in this phase (non-goal; sequence_option = NON).  @return int: -1 """
        self.log.error('Sequence mode is not implemented in this AWG5014C module '
                       '(non-goal; planned phase D+ with the sourced sequence-table limits).')
        return -1

    def get_waveform_names(self):
        """ Waveform names transferred THIS SESSION (local record).

        The instrument-side list query (WLISt:SIZE? / WLISt:NAME?) is not wired up yet —
        its argument syntax page is missing from the in-repo manual extract. Switching
        to the instrument query is a phase-C item (module docstring).
        """
        return natural_sort(self._written_wfm_names)

    def get_sequence_names(self):
        """ Sequence mode is a non-goal this phase.  @return list: empty """
        return list()

    def delete_waveform(self, waveform_name):
        """ Delete waveform(s) from the instrument's waveform list (WLISt:WAVeform:DEL).
        @return list: deleted waveform names """
        if isinstance(waveform_name, str):
            waveform_name = [waveform_name]
        avail = self.get_waveform_names()
        to_delete = [wfm for wfm in waveform_name if wfm in avail]
        for wfm in to_delete:
            self.write('WLIS:WAV:DEL "{0}"'.format(wfm))
        self._written_wfm_names.difference_update(to_delete)
        return to_delete

    def delete_sequence(self, sequence_name):
        """ Sequence mode is a non-goal this phase.  @return list: empty """
        return list()

    def load_waveform(self, load_dict):
        """ Load waveforms from the instrument waveform list into channels via
        SOURce<n>:WAVeform (programmer manual p. 2-23 "how to load a waveform into
        hardware memory"; exact string corroborated by the QCoDeS driver).

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

        # all waveforms must exist in the (session-local) waveform record
        avail = self.get_waveform_names()
        missing = [wfm for wfm in load_dict.values() if wfm not in avail]
        if missing:
            self.log.error('load_waveform: waveform(s) {0} not found on the device.'
                           ''.format(missing))
            return self.get_loaded_assets()[0]

        for ch_num, wfm in load_dict.items():
            self.write('SOUR{0:d}:WAV "{1}"'.format(ch_num, wfm))
            self._loaded_assets[ch_num] = wfm
        self.query('*OPC?')
        return self.get_loaded_assets()[0]

    def load_sequence(self, sequence_name):
        """ NOT AVAILABLE in this phase (non-goal).  @return dict: currently loaded assets """
        self.log.error('Sequence mode is not implemented in this AWG5014C module '
                       '(non-goal; planned phase D+).')
        return self.get_loaded_assets()[0]

    def get_loaded_assets(self):
        """ Internally tracked channel->asset map (cross-check against SOUR<n>:WAVeform?
        readback is a phase-C hardware-validation item).

        @return (dict, str): {channel index: asset name}, 'waveform' (sequences: phase D+)
        """
        return dict(self._loaded_assets), 'waveform'

    def clear_all(self):
        """ Delete all waveforms from the AWG waveform list and forget local bookkeeping.
        @return int: error code (0: OK) """
        self.write('WLIS:WAV:DEL ALL')
        self._written_wfm_names = set()
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
        """ *RST and clear the internal asset bookkeeping.  @return int: 0

        NOTE: whether *RST clears the instrument's user waveform list is unverified
        (phase-C item) — the local waveform record is deliberately KEPT here, matching
        the previous behavior; clear_all() is the explicit wipe.
        """
        self.write('*RST')
        self.query('*OPC?')
        self._loaded_assets = {}
        self._discard_wfm_buffers()
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

    def write_raw(self, message_bytes):
        """ Write a raw bytes message (binary block transfers).  @return int: 0 """
        self.awg.write_raw(message_bytes)
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

    @staticmethod
    def _pack_samples(analog_chunk, marker1=None, marker2=None):
        """ Pack one chunk into the SOURCED integer waveform format:

        Programmer Manual 077-0061-05 Table 2-25 (in-repo extract): 2 bytes/point,
        LSB-first ('<u2'), bit layout M2 M1 D13..D0. Analog scaling (QCoDeS-corroborated,
        module docstring): a in [-1, 1] -> DAC code round(8191 + 8191*a), i.e. 0..16382
        symmetric about mid-code 8191. Marker 1 = bit 14, marker 2 = bit 15.

        @return numpy.ndarray dtype '<u2': packed sample words
        """
        a = np.clip(np.asarray(analog_chunk, dtype=np.float64), -1.0, 1.0)
        words = np.round(8191.0 + 8191.0 * a).astype('<u2')
        if marker1 is not None:
            words |= np.asarray(marker1).astype(bool).astype('<u2') << np.uint16(14)
        if marker2 is not None:
            words |= np.asarray(marker2).astype(bool).astype('<u2') << np.uint16(15)
        return words

    def _transfer_waveform(self, wfm_name):
        """ Send one fully buffered waveform to the instrument:

            WLISt:WAVeform:DEL "<name>"                     (overwrite-safe; no-op if absent)
            WLISt:WAVeform:NEW "<name>",<size>,INTEGER
            WLISt:WAVeform:DATA "<name>",#<nd><nbytes><payload LSB-first>

        Raises ValueError if the accumulated chunk bytes do not add up to the announced
        total_number_of_samples (refuse to send a corrupt waveform).
        """
        payload = b''.join(self._wfm_buffers.pop(wfm_name))
        total = self._wfm_totals.pop(wfm_name)
        n_points = len(payload) // 2
        if n_points != total:
            msg = ('Buffered chunks for waveform "{0}" hold {1:d} samples but '
                   '{2:d} were announced — refusing to transfer a corrupt waveform.'
                   ''.format(wfm_name, n_points, total))
            self.log.error('_transfer_waveform: ' + msg)
            raise ValueError(msg)
        self.write('WLIS:WAV:DEL "{0}"'.format(wfm_name))
        self.write('WLIS:WAV:NEW "{0}",{1:d},INTEGER'.format(wfm_name, n_points))
        nbytes = str(len(payload))
        header = 'WLIS:WAV:DATA "{0}",#{1:d}{2}'.format(wfm_name, len(nbytes), nbytes)
        self.write_raw(header.encode('ascii') + payload)
        self.query('*OPC?')

    def _discard_wfm_buffers(self, prefix=None):
        """ Drop buffered (untransferred) waveform chunks — all of them, or only those of
        one ensemble name ('<prefix>_ch<n>'). """
        if prefix is None:
            self._wfm_buffers = {}
            self._wfm_totals = {}
            return
        for key in [k for k in self._wfm_buffers if k.startswith(prefix + '_ch')]:
            del self._wfm_buffers[key]
            self._wfm_totals.pop(key, None)
