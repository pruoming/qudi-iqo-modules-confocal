# -*- coding: utf-8 -*-

"""
Qudi hardware module: a thin OWNER of a single Swabian Pulse Streamer connection.

To avoid opening two independent connections to one Pulse Streamer, this module owns the connection
and hands the underlying PulseStreamer client to any measurement module that needs to drive it
(confocal_scan_io, odmr_scan_input, ...) via get_pulser(). Only one measurement drives the Pulse
Streamer at a time (a confocal scan OR an ODMR sweep), and each measurement returns it to a safe LOW
state when done, so sharing one client is safe.

SAFETY: sets ALL outputs LOW (laser/mw OFF, no triggers) at activation and deactivation. It never
drives the laser/RF itself — the measurement modules do that through the gated sequences they stream.

This file is part of qudi. Licensed under LGPL v3.
"""

import pulsestreamer as ps

from qudi.core.module import Base
from qudi.core.configoption import ConfigOption


class PulseStreamerProvider(Base):
    """
    Owns one Swabian Pulse Streamer and shares it via get_pulser().

    Example config:

    pulsestreamer_provider:
        module.Class: 'swabian_instruments.pulsestreamer_provider.PulseStreamerProvider'
        options:
            pulsestreamer_ip: '169.254.8.2'
    """
    _ip = ConfigOption(name='pulsestreamer_ip', missing='error')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pulser = None

    def on_activate(self):
        self._pulser = ps.PulseStreamer(self._ip)
        self._pulser.constant(ps.OutputState.ZERO())   # all outputs LOW (laser/mw OFF)
        self.log.info(f'Pulse Streamer connected (owner) at {self._ip}; outputs LOW.')

    def on_deactivate(self):
        if self._pulser is not None:
            try:
                self._pulser.constant(ps.OutputState.ZERO())   # leave all outputs LOW (safe)
            except Exception:
                pass
        self._pulser = None

    def get_pulser(self):
        """ Return the underlying PulseStreamer client so a measurement module can drive the SAME
        device instead of opening a second connection to it. """
        return self._pulser
