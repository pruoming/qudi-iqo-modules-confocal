# -*- coding: utf-8 -*-

"""
Minimal widefield-ODMR GUI — design ladder M3 (widefield_odmr setup, 2026-08-31).

User-preferred layout (design §3.1-M3): side-by-side RF-on | RF-off mean images plus a
contrast readout. NOTHING more — the per-pixel spectrum cursor is M4.
Ledger: Qudi_AI/setups/widefield_odmr/knowledge/fork_changes.yaml.
"""

from PySide6 import QtCore, QtWidgets

from qudi.core.module import GuiBase
from qudi.core.connector import Connector
from qudi.util.widgets.plotting.image_widget import ImageWidget


class WidefieldOdmrMainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle('qudi: Widefield ODMR (M3 — on/off differential)')

        # Toolbar: start/stop + parameters
        toolbar = QtWidgets.QToolBar()
        toolbar.setAllowedAreas(QtCore.Qt.ToolBarArea.AllToolBarAreas)
        self.action_start = toolbar.addAction('Start Burst')
        self.action_stop = toolbar.addAction('Stop')
        toolbar.addSeparator()
        toolbar.addWidget(QtWidgets.QLabel(' pairs: '))
        self.pairs_spinbox = QtWidgets.QSpinBox()
        self.pairs_spinbox.setRange(1, 100000)
        toolbar.addWidget(self.pairs_spinbox)
        toolbar.addWidget(QtWidgets.QLabel(' exposure [ms]: '))
        self.exposure_spinbox = QtWidgets.QDoubleSpinBox()
        self.exposure_spinbox.setRange(1.0, 10000.0)
        self.exposure_spinbox.setValue(30.0)
        toolbar.addWidget(self.exposure_spinbox)
        toolbar.addWidget(QtWidgets.QLabel(' period [ms]: '))
        self.period_spinbox = QtWidgets.QDoubleSpinBox()
        self.period_spinbox.setRange(5.0, 60000.0)
        self.period_spinbox.setValue(100.0)
        toolbar.addWidget(self.period_spinbox)
        self.addToolBar(QtCore.Qt.ToolBarArea.TopToolBarArea, toolbar)

        # Central: RF-on | RF-off images side by side, contrast + progress below
        central = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(central)
        # Layout per operator ruling 2026-08-31: left = MW-off reference,
        # right = per-pixel contrast (on - off) / off.
        images = QtWidgets.QHBoxLayout()
        ref_box = QtWidgets.QVBoxLayout()
        ref_box.addWidget(QtWidgets.QLabel('<b>MW OFF (reference)</b>'))
        self.image_reference = ImageWidget()
        self.image_reference.image_item.setOpts(False, axisOrder='row-major')
        ref_box.addWidget(self.image_reference)
        con_box = QtWidgets.QVBoxLayout()
        con_box.addWidget(QtWidgets.QLabel('<b>(MW ON − OFF) / OFF — contrast</b>'))
        self.image_contrast = ImageWidget()
        self.image_contrast.image_item.setOpts(False, axisOrder='row-major')
        con_box.addWidget(self.image_contrast)
        images.addLayout(ref_box)
        images.addLayout(con_box)
        vbox.addLayout(images)
        bottom = QtWidgets.QHBoxLayout()
        self.contrast_label = QtWidgets.QLabel('mean contrast: —')
        font = self.contrast_label.font()
        font.setPointSize(14)
        font.setBold(True)
        self.contrast_label.setFont(font)
        bottom.addWidget(self.contrast_label)
        bottom.addStretch()
        self.progress_label = QtWidgets.QLabel('idle')
        bottom.addWidget(self.progress_label)
        vbox.addLayout(bottom)
        self.setCentralWidget(central)


class WidefieldOdmrGui(GuiBase):
    """ Minimal M3 GUI: RF-on | RF-off side-by-side + mean contrast.

    Example config for copy-paste:

    widefield_odmr_gui:
        module.Class: 'widefield.widefield_odmr_gui.WidefieldOdmrGui'
        connect:
            widefield_logic: widefield_odmr_logic
    """

    _widefield_logic = Connector(name='widefield_logic', interface='WidefieldOdmrLogic')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mw = None

    def on_activate(self):
        logic = self._widefield_logic()
        self._mw = WidefieldOdmrMainWindow()
        self._mw.pairs_spinbox.setValue(logic.default_n_pairs)

        self._mw.action_stop.setEnabled(False)  # enabled only while a burst runs
        self._mw.action_start.triggered.connect(self._start_clicked)
        # DIRECT connection: the logic thread is busy polling during a burst, so a
        # queued stop would only run after the burst ends. stop_burst just sets a
        # flag, safe to call from the GUI thread.
        self._mw.action_stop.triggered.connect(logic.stop_burst,
                                               QtCore.Qt.DirectConnection)
        logic.sigImagesUpdated.connect(self._update_images)
        logic.sigPairDone.connect(self._update_progress)
        logic.sigBurstFinished.connect(self._burst_finished)
        self.show()

    def on_deactivate(self):
        logic = self._widefield_logic()
        logic.sigBurstFinished.disconnect(self._burst_finished)
        logic.sigPairDone.disconnect(self._update_progress)
        logic.sigImagesUpdated.disconnect(self._update_images)
        self._mw.action_stop.triggered.disconnect()
        self._mw.action_start.triggered.disconnect()
        self._mw.close()

    def show(self):
        self._mw.show()
        self._mw.raise_()
        self._mw.activateWindow()

    def _start_clicked(self):
        logic = self._widefield_logic()
        logic.set_exposure(self._mw.exposure_spinbox.value() / 1000.0)
        logic.set_period(self._mw.period_spinbox.value())
        self._mw.action_start.setEnabled(False)
        self._mw.action_stop.setEnabled(True)
        self._mw.progress_label.setText('starting…')
        logic.start_burst(self._mw.pairs_spinbox.value())

    def _update_images(self, reference_image, contrast_image, mean_contrast):
        self._mw.image_reference.set_image(reference_image)
        self._mw.image_contrast.set_image(contrast_image)
        self._mw.contrast_label.setText(f'mean contrast: {mean_contrast * 100:+.3f} %')

    def _update_progress(self, done, total):
        self._mw.progress_label.setText(f'pair {done}/{total}')

    def _burst_finished(self, ok, message):
        self._mw.action_start.setEnabled(True)
        self._mw.action_stop.setEnabled(False)
        self._mw.progress_label.setText(('done: ' if ok else 'FAILED: ') + message)
