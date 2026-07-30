# -*- coding: utf-8 -*-

"""
Qudi hardware module for the Tektronix (SONY/TEK) AWG2041 arbitrary waveform generator.

FORK MODULE (confocal_odmr, AWG swap 2026-07-23): port of the in-fork
tektronix_awg5014c.py donor (frozen after the 5014C hardware failure, incident AWG-004)
to the AWG2041 replacement. All AWG-001/002/003 lessons are carried over.

INSTRUMENT (sourced: shared_knowledge/device_models.md AWG2041, REQ-033; manuals in-repo
at setups/confocal_odmr/hardware_manuals/):
  * ONE analog channel (CH1 + separate inverted output), 8-bit DAC, clock 1 kHz-1.024 GHz.
  * TWO markers, transferred as their OWN binary block (NOT in the sample word).
  * Waveform length: multiples of 32 points; 32..1M points (4M with Opt 01 — presence on
    this unit to_be_confirmed, REQ-033 answer); >= 640 points once the H/W sequencer is ON.
  * Dialect: IEEE-488.2 common commands + Tek AWG2000-series set over GPIB (no ethernet).
    GPIB0::1::INSTR, IDN 'SONY/TEK,AWG2041,0,CF:91.1CT FV:1.27' (probe 2026-07-23).

SOURCED COMMANDS (AWG2000 Series Programmer Manual, in-repo full PDF; page refs):
  * CLOCk:FREQuency <NR3>  — 1.000000E+3..1.024000E+9 Hz on the AWG2040/41 (p. 2-48).
  * Upload: DATA:DESTination "<name>.WFM" (p. 2-61; overwriting a file that is loaded and
    outputting REPLACES the live output — defined behavior) -> DATA:WIDTh 1 (p. 2-63;
    1 byte/point, native for the 8-bit DAC; DATA:ENCDG only matters at width 2) ->
    CURVe #<x><yyy><binary> (p. 2-59) -> MARKer:DATA #<x><yyy><binary> (p. 2-112;
    1 byte/point, bit0 = marker1, bit1 = marker2, upper 6 bits MUST be 0; goes to the
    same DATA:DESTination file).
  * Analog scaling (Programmer Manual 'Preamble and Curve' / device_models known_quirks):
    width 1 => YOFF fixed at 127; Y = YZERO + (wave - YOFF)*YMULT
    => a in [-1, 1] maps to DAC code round(127 + 127*a), i.e. 0..254 symmetric about 127.
  * Load: [CH1:]WAVeform "<name>.WFM" (p. 2-46). Output enable:
    OUTPut:CH1:NORMal:STATe {ON|OFF|<NR1>} (p. 2-149, AWG2040/41-specific).
  * Run: STARt (p. 2-159) / STOP (p. 2-160) / RUNNing? -> 1|0 (p. 2-152) / MODE?
    (p. 2-141). AWG-010 (measured 2026-07-30): in CONTINUOUS mode STOP is rejected
    (event 221) and RUNNing? sticks at 1 (engine, not connector) — run control is
    MODE-aware, off = output relay(s); CH1 has TWO connectors (NORMal + INVerted,
    pp. 2-147..149), each with its own relay.
  * Levels: [CH1:]AMPLitude 0.020..2.000 V (p. 2-36), [CH1:]OFFSet (p. 2-42; spec range
    -1.000..+1.000 V); markers: [CH1:]MARKERLEVEL1|2:HIGH|LOW (pp. 2-39..2-42,
    AWG2040/41-specific; spec -2.0..+2.0 V into 50 ohm, 0.1 V resolution).
  * File delete: MEMory:DELete {All | <File Name>} (p. 2-123). clear_all deliberately
    deletes ONLY this session's waveforms by name — never 'All' (the internal memory can
    hold unrelated user files).
  * Response headers: device-specific queries prefix responses with the command header
    (':RUNNING 1') unless disabled — HEADer OFF (p. 2-105). on_activate sends this ONE
    deliberate write (communication format only, no signal-path effect); replies are
    additionally parsed header-tolerantly in case headers come back on.
  * Event/error queue (no SYSTem:ERRor? on this dialect): *ESR? latches events, then
    ALLEv? dequeues all event codes+messages (p. 2-30). Helper: drain_events().

CARRIED-OVER LESSONS (known_issues IDs, from the 5014C bring-up):
  * AWG-001: open the VISA resource DIRECTLY — never gate on list_resources().
  * AWG-002 pattern: output-enable INTENT bookkeeping — whether the 2041 refuses
    OUTPut ON without a loaded waveform is UNVERIFIED (phase-C item); the deferred-ON
    pattern is kept because it is correct for both behaviors.
  * AWG-003 pattern: run-state guards — no waveform upload/delete while running. (The
    2041 manual DEFINES overwrite-while-outputting, but the guard stays: qudi's cleanup
    deletes are not needed mid-run, and the 5014C died in exactly that corner.)

PHASE-C VERIFICATION ITEMS (first instrument contact):
  * FILE NAME LENGTH — MEASURED 2026-07-24 (AWG-006): names longer than 8+3 are
    REJECTED with event 257 "File name error; file name too long", and a CURVe sent
    after such a failed DATA:DESTination WEDGES the GPIB bus (the AWG-005 symptom).
    FIX built in: qudi waveform names are mapped to 8.3-safe instrument file names
    (_file_name_for; uppercase, [A-Z0-9_-], body <= 8 chars, collision counter);
    qudi-facing names (get_waveform_names/load/delete) stay the full names.
  * OUTPut ON without waveform: record accept/refuse (AWG-002 pattern above).
  * Whether *RST clears the internal-memory user files is unverified (reset() keeps the
    local record, matching the 5014C policy; clear_all() is the explicit wipe).
  * Drain *ESR?/ALLEv? after every upload step; scope-verify output.
  * MODE is deliberately NEVER touched by this module (front-panel/human decision;
    first light expects CONTinuous — MODE? query is fair game for diagnostics).

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

import time
from collections import OrderedDict

import numpy as np

try:
    import pyvisa as visa
except ImportError:
    import visa

from qudi.core.configoption import ConfigOption
from qudi.util.helpers import natural_sort
from qudi.util.mutex import RecursiveMutex
from qudi.interface.pulser_interface import PulserInterface, PulserConstraints, SequenceOption


class _BusWedgeRecovered(RuntimeError):
    """ Internal signal (AWG-009): the bus wedged after a binary block and a GPIB
    Selected Device Clear brought it back — the enclosing transfer should be retried.
    SDC recovery was MEASURED on a live wedge 2026-07-24 (console clear() -> *ESR? '0'
    with no power-cycle). """
    pass


class AWG2041(PulserInterface):
    """ Tektronix AWG2041: 1 analog channel (CH1), 2 markers (d_ch1 = marker 1,
    d_ch2 = marker 2). Markers are separate binary blocks, NOT sample-word bits.

    Example config for copy-paste:

    pulser_awg2041:
        module.Class: 'awg.tektronix_awg2041.AWG2041'
        options:
            awg_visa_address: 'GPIB0::1::INSTR'   # probe-confirmed 2026-07-23
            timeout: 30                            # VISA timeout in seconds
            # default_sample_rate: 1.024e9  # applied on activation ONLY if set
            # --- constraint overrides; defaults are SOURCED (device_models.md AWG2041) ---
            # sample_rate_min: 1.0e3
            # sample_rate_max: 1.024e9
            # min_waveform_length: 32
            # waveform_granularity: 32
            # max_waveform_length: 1000000   # base unit; 4000000 with Opt 01 (tbc REQ-033)
    """

    # ---- transport config ----
    _visa_address = ConfigOption(name='awg_visa_address', missing='error')
    _visa_timeout = ConfigOption(name='timeout', default=30, missing='warn')  # seconds
    _default_sample_rate = ConfigOption(name='default_sample_rate', default=None,
                                        missing='nothing')

    # ---- device constraints — SOURCED defaults (device_models.md AWG2041 / User Manual
    #      App. B + Programmer Manual p. 2-48), overridable per unit ----
    _cfg_sample_rate_min = ConfigOption(name='sample_rate_min', default=1.0e3,
                                        missing='nothing')   # p. 2-48
    _cfg_sample_rate_max = ConfigOption(name='sample_rate_max', default=1.024e9,
                                        missing='nothing')   # p. 2-48
    _cfg_min_wfm_length = ConfigOption(name='min_waveform_length', default=32,
                                       missing='nothing')    # 32-way multiplexed memory
    _cfg_wfm_granularity = ConfigOption(name='waveform_granularity', default=32,
                                        missing='nothing')   # multiples of 32
    _cfg_max_wfm_length = ConfigOption(name='max_waveform_length', default=1_000_000,
                                       missing='nothing')    # base unit; Opt 01: 4M (tbc)

    # channel topology (structural: one channel, two markers)
    __analog_channels = ('a_ch1',)
    __digital_channels = ('d_ch1', 'd_ch2')

    # Post-block poll budget in seconds (class attribute for shakeout testability;
    # AWG-007/009). Transfers that exceed it trigger the Device-Clear recovery.
    _post_block_timeout_s = 30

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._rm = None          # pyvisa ResourceManager (created on activation)
        self.awg = None          # pyvisa resource
        self.awg_model = ''      # from *IDN?
        self._loaded_assets = {}         # {1: asset name} (internal tracking)
        self._written_wfm_names = set()  # local record of waveforms transferred this session
        self._wfm_buffers = {}   # {wfm_name: {'a': [bytes,...], 'm': [bytes,...]}}
        self._wfm_totals = {}    # {wfm_name: expected total number of samples}
        self._intended_outputs = {1: False}  # AWG-002 pattern (see get_active_channels)
        self._file_names = {}    # {qudi wfm name: 8.3-safe instrument file body} (AWG-006)
        # AWG-008: ALL VISA traffic is serialized by this lock. Qudi is multi-threaded
        # (the pulsed logic polls pulser status on a timer); an unsynchronized query
        # landing inside a transfer sequence hands this instrument a new message while
        # its previous response is pending (its manual's event 420 "Query
        # INTERRUPTED/UNTERMINATED" territory) — consistent with the randomly-located
        # bus wedges seen ONLY inside qudi (2026-07-24) while every single-threaded
        # probe passes. The lock is recursive: multi-command sequences hold it across
        # their whole critical section.
        self._comm_lock = RecursiveMutex()

    # =========================================================================
    # Activation / deactivation
    # =========================================================================

    def on_activate(self):
        """ Connect via VISA/GPIB, verify identity, disable response headers.

        PASSIVE except for two deliberate items: 'HEADer OFF' (communication format
        only — makes query replies parseable; no signal-path effect) and the sample
        rate IF 'default_sample_rate' is explicitly configured by the owner.
        Open DIRECTLY, no list_resources() gate (AWG-001).
        """
        self._rm = visa.ResourceManager()
        try:
            self.awg = self._rm.open_resource(self._visa_address)
        except Exception as err:
            self.awg = None
            try:
                listed = self._rm.list_resources()
            except Exception:
                listed = ('<list_resources() itself failed>',)
            raise RuntimeError(
                'Could not open VISA resource "{0}" ({1}). Check the GPIB cable/address '
                '(front panel UTILITY menu; probe: GPIB0::1::INSTR 2026-07-23). Resources '
                'the manager CAN enumerate: {2}. See connections.yaml devices.awg.'
                ''.format(self._visa_address, err, listed)) from err
        self.awg.timeout = int(self._visa_timeout * 1000)  # pyvisa timeout is in ms

        idn = self.query('*IDN?')
        if 'AWG2041' not in idn.replace(' ', ''):
            self.log.warning('Unexpected *IDN? response (no "AWG2041"): "{0}". Wrong GPIB '
                             'address? Proceeding, but check connections.yaml devices.awg.'
                             ''.format(idn))
        parts = idn.split(',')
        self.awg_model = parts[1].strip() if len(parts) > 1 else idn
        self.log.info('Connected to: {0}'.format(idn))

        # Disable response headers (p. 2-105) — the ONE deliberate settings write at
        # activation (communication format only). Queries are parsed header-tolerantly
        # anyway (see _parse_value).
        self.write('HEADer OFF')

        # Sync the intended-output bookkeeping FROM the instrument (AWG-002 pattern).
        try:
            self._intended_outputs[1] = bool(
                int(self._parse_value(self.query('OUTPut:CH1:NORMal:STATe?'))))
        except Exception:
            self._intended_outputs[1] = False

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
        """ PulserConstraints for the AWG2041 — defaults SOURCED (device_models.md
        AWG2041: User Manual Appendix B + Programmer Manual page refs in the module
        docstring). ConfigOptions allow per-unit overrides (e.g. Opt 01 memory).
        """
        constraints = PulserConstraints()

        constraints.sample_rate.min = float(self._cfg_sample_rate_min)   # p. 2-48
        constraints.sample_rate.max = float(self._cfg_sample_rate_max)   # p. 2-48
        constraints.sample_rate.step = 1.0   # spec: 7-digit resolution — 1 Hz is safe
        constraints.sample_rate.default = float(self._cfg_sample_rate_max)

        # analog level — App. B: 20 mV..2 V p-p into 50 ohm, 1 mV steps; offset ±1 V
        constraints.a_ch_amplitude.min = 0.02
        constraints.a_ch_amplitude.max = 2.0
        constraints.a_ch_amplitude.step = 0.001
        constraints.a_ch_amplitude.default = 2.0

        constraints.a_ch_offset.min = -1.0
        constraints.a_ch_offset.max = 1.0
        constraints.a_ch_offset.step = 0.001
        constraints.a_ch_offset.default = 0.0

        # marker levels — App. B: -2.0..+2.0 V into 50 ohm, 0.1 V resolution
        # (defaults below are OUR choice, not a datasheet value: 0 -> 1 V swing)
        constraints.d_ch_low.min = -2.0
        constraints.d_ch_low.max = 2.0
        constraints.d_ch_low.step = 0.1
        constraints.d_ch_low.default = 0.0

        constraints.d_ch_high.min = -2.0
        constraints.d_ch_high.max = 2.0
        constraints.d_ch_high.step = 0.1
        constraints.d_ch_high.default = 1.0

        constraints.waveform_length.min = int(self._cfg_min_wfm_length)    # 32
        constraints.waveform_length.max = int(self._cfg_max_wfm_length)    # 1M (4M Opt 01)
        constraints.waveform_length.step = int(self._cfg_wfm_granularity)  # 32
        constraints.waveform_length.default = int(self._cfg_min_wfm_length)

        # Number of stored waveforms: NOT a datasheet figure (file-system/memory-limited).
        # Conservative placeholder — revisit if a real limit surfaces at phase C.
        constraints.waveform_num.min = 1
        constraints.waveform_num.max = 100
        constraints.waveform_num.step = 1
        constraints.waveform_num.default = 1

        # Sequence mode is a NON-GOAL for now (H/W sequencer: 5460 steps, >=640-pt
        # waveforms — sourced, but deliberately not implemented yet).
        constraints.sequence_option = SequenceOption.NON

        constraints.event_triggers = list()
        constraints.flags = list()

        activation_config = OrderedDict()
        activation_config['ch1_mk'] = frozenset({'a_ch1', 'd_ch1', 'd_ch2'})
        activation_config['ch1_analog'] = frozenset({'a_ch1'})
        constraints.activation_config = activation_config

        return constraints

    # =========================================================================
    # PulserInterface: run control / status
    # =========================================================================

    def _get_mode(self):
        """ MODE? (p. 2-141) -> uppercase mode string, e.g. 'CONTINUOUS'.

        AWG-010: run-control semantics are MODE-dependent on this instrument, so
        pulser_on/pulser_off/get_status must know the mode. The module still never
        SETS the mode (first-light design rule) — it only reads it.
        """
        return str(self._parse_value(self.query('MODE?'))).strip().upper()

    def pulser_on(self):
        """ Start waveform output — MODE-aware (AWG-010, measured 2026-07-30).

        In CONTINUOUS mode the playback engine auto-runs (RUNNing? flips to 1 on
        the first STARt / on mode entry and cannot be stopped — see pulser_off);
        the connector is gated by the output RELAY. STARt (p. 2-159) is accepted
        in every mode (measured: no event in CONTINUOUS) and is still sent when
        the engine reports stopped — it is what started the engine after the
        fresh power-on at first light (07-24), and the trigger event in
        triggered modes.

        Defensively re-applies the intended output state first (AWG-002 pattern):
        ON only if a waveform is loaded; an intended-ON channel without one is an
        error.
        """
        with self._comm_lock:   # relay + STARt + status = one critical section
            if self._intended_outputs[1]:
                if self._loaded_assets.get(1):
                    self.write('OUTPut:CH1:NORMal:STATe ON')
                else:
                    self.log.error('pulser_on: CH1 is selected active but has no waveform '
                                   'loaded — enable would be meaningless (AWG-002 pattern). '
                                   'Load an asset first.')
            try:
                engine_running = bool(int(self._parse_value(self.query('RUNNing?'))))
            except Exception:
                engine_running = False
            if not engine_running:
                self.write('STARt')
                self._assert_no_error_events('STARt (pulser_on)')
            return self.get_status()[0]

    def pulser_off(self):
        """ Stop waveform output — MODE-aware (AWG-010, measured 2026-07-30).

        CONTINUOUS mode: STOP is REJECTED with event 221 'Settings conflict' and
        the engine cannot be halted (a TRIGGERED+STOP round trip reads back 0 but
        the engine restarts on re-entering CONTINUOUS). The honest off-switch is
        the output RELAY: OUTPut:CH1:NORMal:STATe OFF (p. 2-149; front-panel LED
        confirms the actuation). The INVerted relay (p. 2-147 — the SECOND CH1
        connector on AWG2040/41) is also opened if unexpectedly closed. CAUTION:
        never follow a relay-off with a mode change — re-entering CONTINUOUS was
        observed to re-close relays. Intended-output bookkeeping (AWG-002) is NOT
        changed here: pulser_on re-closes the relay for intended-active channels.

        Other modes: STOP (p. 2-160) works and also resets the sequence pointer.

        @return int: qudi status
        """
        with self._comm_lock:   # mode read + off sequence = one critical section
            if self._get_mode().startswith('CONT'):
                self.write('OUTPut:CH1:NORMal:STATe OFF')
                try:
                    inv_on = bool(int(self._parse_value(self.query('OUTPut:CH1:INVerted?'))))
                except Exception:
                    inv_on = False
                if inv_on:
                    self.log.warning('pulser_off: CH1 INVERTED output relay was ON — the '
                                     'records say nothing hangs on that connector (check '
                                     'the setup / connections.yaml). Opening it too.')
                    self.write('OUTPut:CH1:INVerted:STATe OFF')
                self._assert_no_error_events('output relay off (pulser_off)')
            else:
                self.write('STOP')
                self._assert_no_error_events('STOP (pulser_off)')
            return self.get_status()[0]

    def get_status(self):
        """ Effective run state — MODE-aware (AWG-010, measured 2026-07-30).

        RUNNing? (p. 2-152) reports the playback ENGINE, which in CONTINUOUS mode
        is permanently 1 once started and cannot be stopped (see pulser_off) —
        with the output relays open it still answers 1 (measured; the manual's
        "whether a waveform is being output" wording notwithstanding). The qudi
        semantic 'running' = producing output at the connector, so in CONTINUOUS
        mode this reports 1 only if the engine runs AND an output relay is
        closed. This is also what re-opens qudi's settings gate: the logic
        refuses set_pulse_generator_settings while this reports 1.

        @return (int, dict): current status and description dict (-1 = comms failure)
        """
        status_dic = {-1: 'Failed request or failed communication with device.',
                      0: 'Device has stopped, but can receive commands.',
                      1: 'Device is active and running.'}
        with self._comm_lock:   # multi-query status read = one critical section
            try:
                engine = bool(int(self._parse_value(self.query('RUNNing?'))))
                if engine and self._get_mode().startswith('CONT'):
                    normal_on = bool(int(self._parse_value(
                        self.query('OUTPut:CH1:NORMal?'))))
                    inverted_on = bool(int(self._parse_value(
                        self.query('OUTPut:CH1:INVerted?'))))
                    return (1 if (normal_on or inverted_on) else 0), status_dic
            except Exception:
                return -1, status_dic
        return (1 if engine else 0), status_dic

    # =========================================================================
    # PulserInterface: sample rate
    # =========================================================================

    def get_sample_rate(self):
        """ @return float: current clock frequency from the device (Hz; CLOCk:FREQuency?) """
        return float(self._parse_value(self.query('CLOCk:FREQuency?')))

    def set_sample_rate(self, sample_rate):
        """ Set the clock frequency (Hz; p. 2-48); returns the device read-back. """
        constraints = self.get_constraints()
        sample_rate = float(sample_rate)
        if not (constraints.sample_rate.min <= sample_rate <= constraints.sample_rate.max):
            self.log.error('Requested sample rate {0:.4g} Hz is outside the allowed range '
                           '[{1:.4g}, {2:.4g}] Hz (device_models.md AWG2041). Command '
                           'ignored.'.format(sample_rate, constraints.sample_rate.min,
                                             constraints.sample_rate.max))
            return self.get_sample_rate()
        self.write('CLOCk:FREQuency {0:.7e}'.format(sample_rate))
        self.query('*OPC?')
        return self.get_sample_rate()

    # =========================================================================
    # PulserInterface: analog / digital levels
    # =========================================================================

    def get_analog_level(self, amplitude=None, offset=None):
        """ CH1 amplitude (Vpp; [CH1:]AMPLitude?) and offset (V; [CH1:]OFFSet?). """
        amp_query = list(self.__analog_channels) if not amplitude else list(amplitude)
        off_query = list(self.__analog_channels) if not offset else list(offset)
        amp = {}
        off = {}
        for a_ch in amp_query:
            self._analog_ch_num(a_ch)  # validates
            amp[a_ch] = float(self._parse_value(self.query('CH1:AMPLitude?')))
        for a_ch in off_query:
            self._analog_ch_num(a_ch)
            off[a_ch] = float(self._parse_value(self.query('CH1:OFFSet?')))
        return amp, off

    def set_analog_level(self, amplitude=None, offset=None):
        """ Set CH1 amplitude (Vpp) / offset (V); ranges per Tech spec (App. B). """
        if amplitude is None:
            amplitude = {}
        if offset is None:
            offset = {}
        constraints = self.get_constraints()

        for a_ch, value in amplitude.items():
            self._analog_ch_num(a_ch)
            constr = constraints.a_ch_amplitude
            if not (constr.min <= value <= constr.max):
                self.log.warning('Amplitude {0} Vpp for {1} outside [{2}, {3}] Vpp '
                                 '(AWG2041 App. B). Command ignored.'
                                 ''.format(value, a_ch, constr.min, constr.max))
                continue
            self.write('CH1:AMPLitude {0:.3f}'.format(value))
        for a_ch, value in offset.items():
            self._analog_ch_num(a_ch)
            constr = constraints.a_ch_offset
            if not (constr.min <= value <= constr.max):
                self.log.warning('Offset {0} V for {1} outside [{2}, {3}] V (AWG2041 '
                                 'App. B). Command ignored.'
                                 ''.format(value, a_ch, constr.min, constr.max))
                continue
            self.write('CH1:OFFSet {0:.3f}'.format(value))
        return self.get_analog_level(amplitude=list(amplitude), offset=list(offset))

    def get_digital_level(self, low=None, high=None):
        """ Marker low/high levels (V): [CH1:]MARKERLEVEL<m>:LOW/HIGH (pp. 2-39..2-42);
        d_ch1 -> MARKERLEVEL1, d_ch2 -> MARKERLEVEL2. """
        low_query = list(self.__digital_channels) if not low else list(low)
        high_query = list(self.__digital_channels) if not high else list(high)
        low_val = {}
        high_val = {}
        for d_ch in low_query:
            mrk = self._marker_num(d_ch)
            low_val[d_ch] = float(self._parse_value(
                self.query('CH1:MARKERLEVEL{0:d}:LOW?'.format(mrk))))
        for d_ch in high_query:
            mrk = self._marker_num(d_ch)
            high_val[d_ch] = float(self._parse_value(
                self.query('CH1:MARKERLEVEL{0:d}:HIGH?'.format(mrk))))
        return low_val, high_val

    def set_digital_level(self, low=None, high=None):
        """ Set marker low/high levels (V); spec range ±2.0 V into 50 ohm, 0.1 V steps. """
        if low is None:
            low = {}
        if high is None:
            high = {}
        constraints = self.get_constraints()

        for d_ch, value in low.items():
            mrk = self._marker_num(d_ch)
            constr = constraints.d_ch_low
            if not (constr.min <= value <= constr.max):
                self.log.warning('Marker low {0} V for {1} outside [{2}, {3}] V (AWG2041 '
                                 'App. B). Command ignored.'
                                 ''.format(value, d_ch, constr.min, constr.max))
                continue
            self.write('CH1:MARKERLEVEL{0:d}:LOW {1:.1f}'.format(mrk, value))
        for d_ch, value in high.items():
            mrk = self._marker_num(d_ch)
            constr = constraints.d_ch_high
            if not (constr.min <= value <= constr.max):
                self.log.warning('Marker high {0} V for {1} outside [{2}, {3}] V (AWG2041 '
                                 'App. B). Command ignored.'
                                 ''.format(value, d_ch, constr.min, constr.max))
                continue
            self.write('CH1:MARKERLEVEL{0:d}:HIGH {1:.1f}'.format(mrk, value))
        return self.get_digital_level(low=list(low), high=list(high))

    # =========================================================================
    # PulserInterface: channel activation
    # =========================================================================

    def get_active_channels(self, ch=None):
        """ Active channels = the SELECTED (intended) CH1 state; markers follow CH1
        (they have no independent output switch on this instrument).

        AWG-002 PATTERN (deliberate, documented): reports the module's intended
        selection, not a live readback — whether the 2041 refuses OUTPut ON without a
        loaded waveform is unverified (phase-C item), and qudi's sequence_generator
        applies the activation config before any waveform exists. Intent is synced FROM
        the hardware at activation and reset(), and applied when legal.
        """
        if ch is None:
            ch = list(self.__analog_channels) + list(self.__digital_channels)
        active_ch = {}
        for channel in ch:
            if channel in self.__analog_channels or channel in self.__digital_channels:
                active_ch[channel] = self._intended_outputs[1]
            else:
                raise ValueError('Unknown channel descriptor "{0}" for AWG2041 (valid: '
                                 'a_ch1, d_ch1, d_ch2).'.format(channel))
        return active_ch

    def set_active_channels(self, ch=None):
        """ Select CH1 on/off. OFF applies immediately; ON immediately only if a
        waveform is loaded, else deferred to load_waveform()/pulser_on() (AWG-002
        pattern). Marker entries are accepted but ignored (markers follow CH1). """
        if ch is None:
            ch = {}
        for channel, state in ch.items():
            if channel in self.__analog_channels:
                self._intended_outputs[1] = bool(state)
                if not state:
                    self.write('OUTPut:CH1:NORMal:STATe OFF')
                elif self._loaded_assets.get(1):
                    self.write('OUTPut:CH1:NORMal:STATe ON')
                else:
                    self.log.debug('CH1 ON deferred until a waveform is loaded '
                                   '(AWG-002 pattern).')
            elif channel in self.__digital_channels:
                self.log.debug('Marker channel {0} follows CH1 on the AWG2041; entry '
                               'ignored.'.format(channel))
            else:
                raise ValueError('Unknown channel descriptor "{0}" for AWG2041 (valid: '
                                 'a_ch1, d_ch1, d_ch2).'.format(channel))
        return self.get_active_channels()

    # =========================================================================
    # PulserInterface: waveform write / load / bookkeeping
    # =========================================================================

    def write_waveform(self, name, analog_samples, digital_samples, is_first_chunk,
                       is_last_chunk, total_number_of_samples):
        """ Pack samples into the sourced CURVe width-1 format (analog) + the separate
        MARKer:DATA format, buffer qudi's chunks locally, and transfer ONE waveform file
        per call series: DATA:DESTination -> DATA:WIDTh 1 -> CURVe -> MARKer:DATA.

        ERROR CONTRACT (shakeout-validated, 5014C lineage): illegal total length (below
        min / above max / granularity violation — the 2041 wants multiples of 32; the
        instrument would silently ZERO-PAD, we refuse instead) and unknown channel
        descriptors RAISE ValueError before anything is buffered. Nothing reaches the
        instrument before the last chunk. Refuses while running (AWG-003 pattern).

        @return (int, list): samples consumed this chunk (-1 on failure) and waveform names
        """
        waveforms = list()

        unknown = [k for k in analog_samples if k not in self.__analog_channels]
        unknown += [k for k in digital_samples if k not in self.__digital_channels]
        if unknown:
            self.log.error('write_waveform: unknown channel descriptor(s) {0} for AWG2041.'
                           ''.format(unknown))
            raise ValueError('Unknown channel descriptor(s) for AWG2041: {0} (valid: '
                             'a_ch1, d_ch1, d_ch2).'.format(unknown))

        if len(analog_samples) == 0:
            self.log.error('write_waveform: no analog samples passed (the AWG2041 '
                           'waveform file is built around the CH1 analog data).')
            return -1, waveforms

        if is_first_chunk:
            if self.get_status()[0] != 0:
                msg = ('write_waveform: the AWG is running (or unreachable) — stop the '
                       'pulser before sampling/rewriting waveforms (AWG-003 pattern).')
                self.log.error(msg)
                raise RuntimeError(msg)
            constraints = self.get_constraints()
            total = int(total_number_of_samples)
            if total < constraints.waveform_length.min:
                msg = ('Waveform length {0:d} is below the minimum of {1:d} samples '
                       '(32-way multiplexed memory).'
                       ''.format(total, constraints.waveform_length.min))
                self.log.error('write_waveform: ' + msg)
                raise ValueError(msg)
            if total > constraints.waveform_length.max:
                msg = ('Waveform length {0:d} exceeds the maximum of {1:d} samples '
                       '(base-unit memory; Opt 01 presence tbc — REQ-033).'
                       ''.format(total, constraints.waveform_length.max))
                self.log.error('write_waveform: ' + msg)
                raise ValueError(msg)
            granularity = constraints.waveform_length.step
            if total % granularity != 0:
                msg = ('Waveform length {0:d} violates the length granularity {1:d} — '
                       'the instrument would silently zero-pad; refusing instead. Pad '
                       'the ensemble deliberately.'.format(total, granularity))
                self.log.error('write_waveform: ' + msg)
                raise ValueError(msg)

        chunk_length = len(analog_samples[list(analog_samples)[0]])
        for chnl, samples in list(analog_samples.items()) + list(digital_samples.items()):
            if len(samples) != chunk_length:
                self.log.error('write_waveform: unequal sample array lengths across channels.')
                self._discard_wfm_buffers(prefix=name)
                return -1, waveforms

        activation = self.get_active_channels()
        active_analog = natural_sort(c for c in self.__analog_channels if activation[c])
        if set(analog_samples) != set(active_analog):
            self.log.error('write_waveform: mismatch between active analog channels {0} '
                           'and provided sample arrays {1}.'
                           ''.format(active_analog, sorted(analog_samples)))
            self._discard_wfm_buffers(prefix=name)
            return -1, waveforms

        wfm_name = '{0}_ch1'.format(name)
        analog_bytes = self._pack_analog(analog_samples['a_ch1'])
        marker_bytes = self._pack_markers(chunk_length,
                                          digital_samples.get('d_ch1'),
                                          digital_samples.get('d_ch2'))
        if is_first_chunk:
            self._wfm_buffers[wfm_name] = {'a': [], 'm': []}
            self._wfm_totals[wfm_name] = int(total_number_of_samples)
        self._wfm_buffers[wfm_name]['a'].append(analog_bytes.tobytes())
        self._wfm_buffers[wfm_name]['m'].append(marker_bytes.tobytes())

        if is_last_chunk:
            self._transfer_waveform(wfm_name)
            self._written_wfm_names.add(wfm_name)
        waveforms.append(wfm_name)

        # sequence_generator_logic compares the return value against the samples staged
        # in THIS call (chunk) -> return the chunk length.
        return chunk_length, waveforms

    def write_sequence(self, name, sequence_parameters):
        """ NOT AVAILABLE yet (sequence_option = NON; H/W sequencer is a later phase).
        @return int: -1 """
        self.log.error('Sequence mode is not implemented in this AWG2041 module yet '
                       '(H/W sequencer: 5460 steps, >=640-pt waveforms — later phase).')
        return -1

    def get_waveform_names(self):
        """ Waveform names transferred THIS SESSION (local record; '.WFM' stripped).
        Instrument-side listing (MEMory catalog) is a phase-C upgrade item. """
        return natural_sort(self._written_wfm_names)

    def get_sequence_names(self):
        """ Sequence mode not implemented.  @return list: empty """
        return list()

    def delete_waveform(self, waveform_name):
        """ Delete waveform file(s) from the internal memory (MEMory:DELete, p. 2-123).
        Refuses while running (AWG-003 pattern; called from cleanup code — no raise).
        @return list: deleted waveform names """
        if isinstance(waveform_name, str):
            waveform_name = [waveform_name]
        if self.get_status()[0] != 0:
            self.log.error('delete_waveform: the AWG is running (or unreachable) — '
                           'refusing to delete files (AWG-003 pattern). Stop the pulser '
                           'first. Nothing deleted.')
            return list()
        avail = self.get_waveform_names()
        to_delete = [wfm for wfm in waveform_name if wfm in avail]
        for wfm in to_delete:
            self.write('MEMory:DELete "{0}.WFM"'.format(self._file_name_for(wfm)))
            self._file_names.pop(wfm, None)
        self._written_wfm_names.difference_update(to_delete)
        return to_delete

    def delete_sequence(self, sequence_name):
        """ Sequence mode not implemented.  @return list: empty """
        return list()

    def load_waveform(self, load_dict):
        """ Load a waveform file onto CH1 via [CH1:]WAVeform "<name>.WFM" (p. 2-46).

        @param dict|list load_dict: {1: waveform name} or list of names with '_ch1'
        @return dict: actually loaded waveforms per channel
        """
        if isinstance(load_dict, list):
            new_dict = dict()
            for waveform in load_dict:
                channel = int(waveform.rsplit('_ch', 1)[1])
                new_dict[channel] = waveform
            load_dict = new_dict

        invalid = [ch for ch in load_dict if ch != 1]
        if invalid:
            self.log.error('load_waveform: invalid channel index(es) {0} (the AWG2041 '
                           'has ONE channel: 1).'.format(invalid))
            raise ValueError('Invalid AWG2041 channel index(es): {0} (valid: 1).'
                             ''.format(invalid))

        avail = self.get_waveform_names()
        missing = [wfm for wfm in load_dict.values() if wfm not in avail]
        if missing:
            self.log.error('load_waveform: waveform(s) {0} not found on the device.'
                           ''.format(missing))
            return self.get_loaded_assets()[0]

        with self._comm_lock:   # load + deferred enable = one critical section (AWG-008)
            for attempt in range(1, 3):
                try:
                    for ch_num, wfm in load_dict.items():
                        self.write('CH1:WAVeform "{0}.WFM"'
                                   ''.format(self._file_name_for(wfm)))
                        self._loaded_assets[ch_num] = wfm
                    # Probe-proven sync (AWG-007); loading a big file into waveform
                    # memory may take a while -> generous budget. Device-Clear retry
                    # on the intermittent wedge (AWG-009).
                    self._wait_after_block('CH1:WAVeform load',
                                           str(list(load_dict.values())), max_s=60)
                    break
                except _BusWedgeRecovered as wedge:
                    self.log.warning('Load attempt {0:d}/2: {1} — retrying.'
                                     ''.format(attempt, wedge))
            else:
                raise RuntimeError('Waveform load wedged on 2 consecutive attempts '
                                   '(bus recovered each time) — giving up (AWG-009).')
        # Apply a deferred output-enable intent now that a waveform exists (AWG-002).
        if self._intended_outputs.get(1):
            self.write('OUTPut:CH1:NORMal:STATe ON')
            self.query('*OPC?')
        return self.get_loaded_assets()[0]

    def load_sequence(self, sequence_name):
        """ NOT AVAILABLE yet.  @return dict: currently loaded assets """
        self.log.error('Sequence mode is not implemented in this AWG2041 module yet.')
        return self.get_loaded_assets()[0]

    def get_loaded_assets(self):
        """ Internally tracked channel->asset map (CH1:WAVeform? readback cross-check is
        a phase-C item).  @return (dict, str): {1: asset name}, 'waveform' """
        return dict(self._loaded_assets), 'waveform'

    def clear_all(self):
        """ Delete THIS SESSION'S waveform files from internal memory and forget local
        bookkeeping. Deliberately NOT 'MEMory:DELete All' — internal memory can hold
        unrelated user files.  @return int: 0 OK, -1 refused while running """
        if self.get_status()[0] != 0:
            self.log.error('clear_all: the AWG is running (or unreachable) — refusing '
                           '(AWG-003 pattern). Stop the pulser first.')
            return -1
        for wfm in list(self._written_wfm_names):
            self.write('MEMory:DELete "{0}.WFM"'.format(self._file_name_for(wfm)))
        self._written_wfm_names = set()
        self._file_names = {}
        self._loaded_assets = {}
        return 0

    # =========================================================================
    # PulserInterface: misc
    # =========================================================================

    def get_interleave(self):
        """ No interleave on the AWG2041; always False. """
        return False

    def set_interleave(self, state=False):
        """ No interleave on the AWG2041; warns if activation is requested. """
        if state:
            self.log.warning('Interleave mode not available for the AWG2041. '
                             'Method call ignored.')
        return False

    def reset(self):
        """ *RST and clear the internal asset bookkeeping.  @return int: 0

        Whether *RST clears the internal-memory user FILES is unverified (phase-C
        item) — the local waveform record is KEPT (5014C policy); clear_all() is the
        explicit wipe. Output intent is re-synced from the instrument.
        """
        self.write('*RST')
        self.query('*OPC?')
        self._loaded_assets = {}
        self._discard_wfm_buffers()
        try:
            self._intended_outputs[1] = bool(
                int(self._parse_value(self.query('OUTPut:CH1:NORMal:STATe?'))))
        except Exception:
            self._intended_outputs[1] = False
        return 0

    # =========================================================================
    # Communication helpers (VISA/GPIB)
    # =========================================================================

    def query(self, question):
        """ Query the device and return the stripped answer string. The write+read pair
        is ATOMIC under the comm lock (AWG-008). """
        with self._comm_lock:
            return self.awg.query(question).strip().strip('"')

    def write(self, command):
        """ Write a command to the device (comm-locked, AWG-008).  @return int: 0 """
        with self._comm_lock:
            self.awg.write(command)
        return 0

    def write_raw(self, message_bytes):
        """ Write a raw bytes message (binary blocks; comm-locked, AWG-008).
        @return int: 0 """
        with self._comm_lock:
            self.awg.write_raw(message_bytes)
        return 0

    def drain_events(self, max_events=30):
        """ Drain the event queue: *ESR? latches, ALLEv? dequeues (p. 2-30). Phase-C
        diagnostic helper (this dialect has no SYSTem:ERRor?).
        @return list of str: event report lines """
        lines = ['*ESR? -> {0}'.format(self.query('*ESR?'))]
        for _ in range(max_events):
            events = self.query('ALLEv?')
            lines.append('ALLEv? -> {0}'.format(events))
            # event code 0 / 'No events' terminates the dialogue
            if events.startswith('0') or 'No events' in events:
                break
        return lines

    # =========================================================================
    # Internal helpers
    # =========================================================================

    @staticmethod
    def _parse_value(response):
        """ Header-tolerant reply parsing: with 'HEADer ON' device queries answer e.g.
        ':RUNNING 1' or ':CLOCK:FREQUENCY 1.024E+9' — return the text after the last
        space; without headers, return the response unchanged. """
        response = response.strip()
        if response.startswith(':') and ' ' in response:
            return response.rsplit(' ', 1)[1]
        return response

    def _drain_error_events(self):
        """ Collect queued event strings (ALLEv?, max 10).  @return list of str """
        events = []
        for _ in range(10):
            ev = self.query('ALLEv?')
            events.append(ev)
            if ev.startswith('0'):
                break
        return events

    def _assert_no_error_events(self, context):
        """ Raise RuntimeError (with the dequeued event texts) if *ESR? shows any of the
        488.2 error bits (CME 32 | EXE 16 | DDE 8 | QYE 4). Non-error bits (PON, OPC,
        URQ) are ignored. """
        esr = int(self._parse_value(self.query('*ESR?')))
        if esr & 0b00111100:
            events = self._drain_error_events()
            msg = ('AWG2041 reported error(s) after {0}: *ESR?={1}, events={2}'
                   ''.format(context, esr, events))
            self.log.error(msg)
            raise RuntimeError(msg)

    def _wait_after_block(self, what, wfm_name, max_s=None):
        """ Probe-proven post-block synchronization (AWG-007): poll *ESR? with SHORT
        timeouts until the instrument answers again (it holds the bus ~0.3-0.4 s after
        a block), then check the error bits. Do NOT use *OPC? here (AWG-007).

        AWG-009 recovery: if the poll budget runs out (the intermittent message-layer
        hang — root cause open, tracked via NI I/O Trace), send a GPIB Selected Device
        Clear and re-poll: SDC recovery is MEASURED to work on a live wedge without a
        power-cycle. On recovery, raise _BusWedgeRecovered so the caller retries the
        whole transfer; only if even SDC fails, raise the fatal power-cycle error. """
        if max_s is None:
            max_s = self._post_block_timeout_s
        old_timeout = self.awg.timeout
        self.awg.timeout = 2000
        t0 = time.time()
        try:
            while True:
                try:
                    with self._comm_lock:   # atomic write+read (AWG-008)
                        esr = int(self._parse_value(
                            self.awg.query('*ESR?').strip().strip('"')))
                    break
                except Exception:
                    if time.time() - t0 <= max_s:
                        continue
                    # AWG-009: budget exhausted -> Device Clear + re-poll
                    self.log.warning('AWG2041: bus stuck after the {0} block for "{1}" '
                                     '(>{2:d} s) — sending GPIB Device Clear (AWG-009).'
                                     ''.format(what, wfm_name, int(max_s)))
                    try:
                        with self._comm_lock:
                            self.awg.clear()
                    except Exception:
                        pass
                    t1 = time.time()
                    recovered = False
                    while time.time() - t1 < 10:
                        try:
                            with self._comm_lock:
                                self.awg.query('*ESR?')
                            recovered = True
                            break
                        except Exception:
                            pass
                    if recovered:
                        raise _BusWedgeRecovered(
                            'bus wedged after the {0} block for "{1}"; recovered via '
                            'GPIB Device Clear (AWG-009)'.format(what, wfm_name))
                    raise RuntimeError(
                        'AWG2041 bus wedged after the {0} block for "{1}" and even '
                        'GPIB Device Clear did not recover it — power-cycle needed '
                        '(AWG-009).'.format(what, wfm_name))
        finally:
            self.awg.timeout = old_timeout
        if esr & 0b00111100:
            events = self._drain_error_events()
            msg = ('AWG2041 rejected the {0} block for "{1}": *ESR?={2}, events={3}'
                   ''.format(what, wfm_name, esr, events))
            self.log.error(msg)
            raise RuntimeError(msg)

    def _file_name_for(self, wfm_name, create=False):
        """ Map a qudi waveform name to an 8.3-SAFE instrument file body (AWG-006,
        measured 2026-07-24: DATA:DESTination rejects longer names with event 257
        'File name error; file name too long', and a CURVe after the failed
        destination WEDGES the GPIB bus). Mapping: uppercase, characters outside
        [A-Z0-9_-] replaced by '_'; if the result exceeds 8 chars (or collides),
        use '<first 5>_<nn>' with a session counter. The mapping is session-local,
        like the waveform record itself.

        @return str: file body (append '.WFM' for the instrument)
        """
        if wfm_name in self._file_names:
            return self._file_names[wfm_name]
        if not create:
            raise ValueError('No instrument file is mapped for waveform "{0}" — it was '
                             'never transferred this session.'.format(wfm_name))
        base = ''.join(ch if (ch.isalnum() or ch in '_-') else '_'
                       for ch in wfm_name.upper())
        taken = set(self._file_names.values())
        if len(base) <= 8 and base not in taken:
            short = base
        else:
            for i in range(1, 100):
                short = '{0}_{1:02d}'.format(base[:5], i)
                if short not in taken:
                    break
            else:
                raise ValueError('No free 8.3-safe file name left for waveform "{0}" '
                                 '(AWG-006).'.format(wfm_name))
            self.log.info('AWG2041: waveform "{0}" stored as "{1}.WFM" (8+3 file-name '
                          'limit, AWG-006).'.format(wfm_name, short))
        self._file_names[wfm_name] = short
        return short

    def _analog_ch_num(self, a_ch):
        """ 'a_ch1' -> 1, validating the descriptor (raise ValueError otherwise). """
        if a_ch not in self.__analog_channels:
            raise ValueError('Unknown analog channel descriptor "{0}" for AWG2041 '
                             '(valid: a_ch1).'.format(a_ch))
        return 1

    def _marker_num(self, d_ch):
        """ 'd_ch1'/'d_ch2' -> marker index 1/2; raises on invalid. """
        if d_ch not in self.__digital_channels:
            raise ValueError('Unknown digital channel descriptor "{0}" for AWG2041 '
                             '(valid: d_ch1, d_ch2).'.format(d_ch))
        return int(d_ch.rsplit('ch', 1)[1])

    @staticmethod
    def _pack_analog(analog_chunk):
        """ Pack one analog chunk into the SOURCED CURVe width-1 format: 1 byte/point,
        YOFF fixed 127 => a in [-1, 1] -> round(127 + 127*a), 0..254 about mid-code 127
        (Programmer Manual 'Preamble and Curve'; device_models AWG2041 known_quirks).
        @return numpy.ndarray dtype u1 """
        a = np.clip(np.asarray(analog_chunk, dtype=np.float64), -1.0, 1.0)
        return np.round(127.0 + 127.0 * a).astype(np.uint8)

    @staticmethod
    def _pack_markers(n_samples, marker1=None, marker2=None):
        """ Pack one marker chunk into the SOURCED MARKer:DATA format: 1 byte/point,
        bit0 = marker 1, bit1 = marker 2, upper 6 bits zero (p. 2-112).
        @return numpy.ndarray dtype u1 """
        out = np.zeros(int(n_samples), dtype=np.uint8)
        if marker1 is not None:
            out |= np.asarray(marker1).astype(bool).astype(np.uint8)
        if marker2 is not None:
            out |= np.asarray(marker2).astype(bool).astype(np.uint8) << np.uint8(1)
        return out

    def _transfer_waveform(self, wfm_name):
        """ Send one fully buffered waveform file to the instrument:

            DATA:DESTination "<name>.WFM"     (p. 2-61; overwrite is defined behavior)
            DATA:WIDTh 1                      (p. 2-63; 1 byte/point, 8-bit native)
            CURVe #<x><yyy><analog bytes>     (p. 2-59)
            MARKer:DATA #<x><yyy><marker bytes>  (p. 2-112; same destination file)

        Raises ValueError if the accumulated bytes do not match the announced total.
        """
        bufs = self._wfm_buffers.pop(wfm_name)
        total = self._wfm_totals.pop(wfm_name)
        analog_payload = b''.join(bufs['a'])
        marker_payload = b''.join(bufs['m'])
        if len(analog_payload) != total or len(marker_payload) != total:
            msg = ('Buffered chunks for waveform "{0}" hold {1:d} analog / {2:d} marker '
                   'bytes but {3:d} samples were announced — refusing to transfer a '
                   'corrupt waveform.'.format(wfm_name, len(analog_payload),
                                              len(marker_payload), total))
            self.log.error('_transfer_waveform: ' + msg)
            raise ValueError(msg)
        file_body = self._file_name_for(wfm_name, create=True)   # 8.3-safe (AWG-006)
        # The WHOLE transfer sequence is one critical section (AWG-008), retried up to
        # 3x with GPIB-Device-Clear recovery on the intermittent wedge (AWG-009).
        with self._comm_lock:
            for attempt in range(1, 4):
                try:
                    # Clean status slate (*CLS) so checks see only errors CAUSED HERE
                    # (attempt 4's 221 mis-attribution). Then: DEST + WIDTh with a
                    # hard-stop error check (AWG-006), LF-terminated blocks (AWG-005),
                    # polled-*ESR? sync (AWG-007).
                    self.write('*CLS')
                    self.write('DATA:DESTination "{0}.WFM"'.format(file_body))
                    self.write('DATA:WIDTh 1')
                    self._assert_no_error_events('DATA:DESTination "{0}.WFM" / '
                                                 'DATA:WIDTh'.format(file_body))
                    self.write_raw(b'CURVe ' + self._block_header(len(analog_payload))
                                   + analog_payload + b'\n')
                    self._wait_after_block('CURVe', wfm_name)
                    self.write_raw(b'MARKer:DATA '
                                   + self._block_header(len(marker_payload))
                                   + marker_payload + b'\n')
                    self._wait_after_block('MARKer:DATA', wfm_name)
                    break
                except _BusWedgeRecovered as wedge:
                    self.log.warning('Transfer attempt {0:d}/3 for "{1}": {2} — '
                                     'retrying the full transfer.'
                                     ''.format(attempt, wfm_name, wedge))
            else:
                raise RuntimeError(
                    'Waveform transfer for "{0}" wedged on 3 consecutive attempts '
                    '(bus recovered via Device Clear each time) — giving up (AWG-009). '
                    'The instrument is left responsive; investigate before retrying.'
                    ''.format(wfm_name))

    @staticmethod
    def _block_header(n_bytes):
        """ IEEE-488.2 arbitrary block header: #<x><yyy> (p. 2-59/2-112). """
        nbytes = str(int(n_bytes))
        return ('#{0:d}{1}'.format(len(nbytes), nbytes)).encode('ascii')

    def _discard_wfm_buffers(self, prefix=None):
        """ Drop buffered (untransferred) waveform chunks — all, or one ensemble's. """
        if prefix is None:
            self._wfm_buffers = {}
            self._wfm_totals = {}
            return
        for key in [k for k in self._wfm_buffers if k.startswith(prefix + '_ch')]:
            del self._wfm_buffers[key]
            self._wfm_totals.pop(key, None)
