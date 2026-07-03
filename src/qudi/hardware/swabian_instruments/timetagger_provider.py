# -*- coding: utf-8 -*-

"""
Qudi hardware module: a thin OWNER of a single Swabian Time Tagger connection.

A Time Tagger allows only ONE createTimeTagger() connection per device, but supports many concurrent
measurements on that one connection. This module owns the connection and hands the underlying
TimeTagger object to any number of measurement modules (confocal_scan_io, odmr_scan_input,
timetagger_instreamer, ...) via get_tagger(), so they all share one device cleanly instead of each
trying to open it (which would fail) or one measurement module doubling as the owner.

SAFETY: read-only with respect to the experiment — it only opens the Time Tagger connection and frees
it on deactivate. It enables no laser, RF, or motion.

This file is part of qudi. Licensed under LGPL v3.
"""

import TimeTagger as tt

from qudi.core.module import Base
from qudi.core.configoption import ConfigOption


class TimeTaggerProvider(Base):
    """
    Owns one Swabian Time Tagger and shares it via get_tagger().

    Example config:

    timetagger_provider:
        module.Class: 'swabian_instruments.timetagger_provider.TimeTaggerProvider'
        options:
            serial: ''    # optional; '' -> auto-connect the only Time Tagger (e.g. '1740000JEC')
    """
    _serial = ConfigOption(name='serial', default='')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._tagger = None

    def on_activate(self):
        self._tagger = tt.createTimeTagger(self._serial) if self._serial else tt.createTimeTagger()
        self.log.info(f'Time Tagger connected (owner), serial {self._tagger.getSerial()}.')

    def on_deactivate(self):
        if self._tagger is not None:
            try:
                tt.freeTimeTagger(self._tagger)
            except Exception:
                pass
        self._tagger = None

    def get_tagger(self):
        """ Return the underlying TimeTagger object. Callers run their own measurements on it; the
        device supports multiple concurrent measurements on this single shared connection. """
        return self._tagger
