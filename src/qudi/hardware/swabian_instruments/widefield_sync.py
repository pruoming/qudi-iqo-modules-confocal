# -*- coding: utf-8 -*-

"""
Thin Pulse Streamer wrapper for widefield-ODMR frame/gate synchronization
(widefield_odmr setup, design ladder M3 — 2026-08-31).

Drives exactly two channels: the camera frame trigger and the MW switch gate.
Deliberately NOT the stock PulserInterface module (waveform machinery is overkill for a
two-channel burst); ledger: Qudi_AI/setups/widefield_odmr/knowledge/fork_changes.yaml.

SAFETY: streaming drives real TTL outputs. The gate channel (default ch4) toggles the
physical MW switch — harmless only while the MW source RF is OFF / unapproved. Every
run in a session must be operator-approved until a standing approval exists
(approved_actions.yaml, human-authority). Idle/final state is always ALL-LOW
(device_models.md Pulse Streamer stream() final-state quirk).
"""

from pulsestreamer import PulseStreamer, OutputState

from qudi.core.module import Base
from qudi.core.configoption import ConfigOption


class WidefieldSync(Base):
    """ Two-channel Pulse Streamer burst generator for camera-synchronized MW gating.

    Example config for copy-paste:

    widefield_sync:
        module.Class: 'swabian_instruments.widefield_sync.WidefieldSync'
        options:
            ip: '169.254.8.2'
            trigger_channel: 1   # camera Trigger In (REQ-006; PS labels 0-indexed)
            gate_channel: 4      # MW switch TTL (HIGH = RF toward amp)
    """

    _ip = ConfigOption(name='ip', default='169.254.8.2', missing='warn')
    _trigger_channel = ConfigOption(name='trigger_channel', default=1)
    _gate_channel = ConfigOption(name='gate_channel', default=4)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ps = None

    def on_activate(self):
        self._ps = PulseStreamer(self._ip)
        self.log.info(f'Pulse Streamer at {self._ip}: serial {self._ps.getSerial()}, '
                      f'firmware {self._ps.getFirmwareVersion()}')

    def on_deactivate(self):
        # Leave outputs in a defined all-LOW state, then drop the client.
        try:
            self.all_low()
        finally:
            self._ps = None

    def run_pair_burst(self, n_pairs, period_ms, trigger_us=10.0, gate_on_first=True,
                       drive_gate=True):
        """ Fire 2*n_pairs camera triggers; the gate channel is HIGH for one frame of
        each pair (alternating), LOW for the other — the M3 RF-on/RF-off pattern.

        M3 minimal: the gate is HIGH for the WHOLE on-frame period. Restricting it to
        the all-rows global-exposure window happens once O7 (expose-out) is verified —
        design §3/§7-O7.

        @param int n_pairs: number of on/off frame PAIRS (total frames = 2*n_pairs)
        @param float period_ms: trigger-to-trigger period (must cover exposure+readout)
        @param float trigger_us: trigger pulse width
        @param bool gate_on_first: True = first frame of each pair is the gated one
        @param bool drive_gate: False = trigger only, gate channel untouched (stays LOW)
        """
        n_pairs = int(n_pairs)
        period_ns = int(round(period_ms * 1e6))
        trig_ns = int(round(trigger_us * 1e3))
        if n_pairs < 1 or period_ns <= trig_ns:
            raise ValueError('Bad burst parameters.')

        seq = self._ps.createSequence()
        trig_pattern = [(trig_ns, 1), (period_ns - trig_ns, 0)] * (2 * n_pairs)
        seq.setDigital(int(self._trigger_channel), trig_pattern)
        if drive_gate:
            on_frame = [(period_ns, 1)]
            off_frame = [(period_ns, 0)]
            pair = (on_frame + off_frame) if gate_on_first else (off_frame + on_frame)
            seq.setDigital(int(self._gate_channel), pair * n_pairs)
        # Final/idle state DELIBERATELY all-LOW (device_models.md stream() quirk).
        self._ps.stream(seq, 1, OutputState.ZERO())
        return 2 * n_pairs

    def has_finished(self):
        """ @return bool: True once the streamed burst has completely played out """
        return bool(self._ps.hasFinished())

    def all_low(self):
        """ Force every output to a constant LOW (defined idle state). """
        self._ps.constant(OutputState.ZERO())
        return True
