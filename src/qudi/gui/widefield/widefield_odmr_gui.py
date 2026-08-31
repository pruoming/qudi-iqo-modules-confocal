# -*- coding: utf-8 -*-

"""
Widefield-ODMR GUI — design ladder M3+M4 (widefield_odmr setup, 2026-08-31).

Layout (operator rulings 2026-08-31): left = MW-off reference image; right = per-pixel
contrast map (on−off)/off. M4 additions: frequency sweep controls, a frequency selector
for the contrast panel, and pixel selection by cursor with the pixel's contrast-vs-
frequency spectrum plotted below.
Ledger: Qudi_AI/setups/widefield_odmr/knowledge/fork_changes.yaml.
"""

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets

from qudi.core.module import GuiBase
from qudi.core.connector import Connector
from qudi.util.widgets.plotting.image_widget import ImageWidget


class WidefieldOdmrMainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle('qudi: Widefield ODMR (M4 — sweep + pixel spectrum)')

        # Toolbar 1: fixed-frequency burst (M3 mode) + common parameters
        toolbar = QtWidgets.QToolBar('Burst')
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
        self.period_spinbox.setValue(250.0)  # CAM-004: > measured frame cycle at speed mode 1
        toolbar.addWidget(self.period_spinbox)
        self.addToolBar(QtCore.Qt.ToolBarArea.TopToolBarArea, toolbar)

        # Toolbar 2 (M4): frequency sweep — on its OWN row (it gets squeezed behind
        # the ">>" chevron otherwise; found the hard way 2026-08-31)
        self.addToolBarBreak(QtCore.Qt.ToolBarArea.TopToolBarArea)
        sweep_bar = QtWidgets.QToolBar('Sweep')
        self.action_start_sweep = sweep_bar.addAction('Start Sweep')
        sweep_bar.addWidget(QtWidgets.QLabel(' f start [GHz]: '))
        self.fstart_spinbox = QtWidgets.QDoubleSpinBox()
        self.fstart_spinbox.setDecimals(4)
        self.fstart_spinbox.setRange(0.1, 20.0)
        self.fstart_spinbox.setValue(2.77)
        sweep_bar.addWidget(self.fstart_spinbox)
        sweep_bar.addWidget(QtWidgets.QLabel(' f stop [GHz]: '))
        self.fstop_spinbox = QtWidgets.QDoubleSpinBox()
        self.fstop_spinbox.setDecimals(4)
        self.fstop_spinbox.setRange(0.1, 20.0)
        self.fstop_spinbox.setValue(2.97)
        sweep_bar.addWidget(self.fstop_spinbox)
        sweep_bar.addWidget(QtWidgets.QLabel(' points: '))
        self.points_spinbox = QtWidgets.QSpinBox()
        self.points_spinbox.setRange(2, 1001)
        self.points_spinbox.setValue(41)
        sweep_bar.addWidget(self.points_spinbox)
        sweep_bar.addWidget(QtWidgets.QLabel(' pairs/point: '))
        self.ppp_spinbox = QtWidgets.QSpinBox()
        self.ppp_spinbox.setRange(1, 10000)
        self.ppp_spinbox.setValue(5)
        sweep_bar.addWidget(self.ppp_spinbox)
        self.addToolBar(QtCore.Qt.ToolBarArea.TopToolBarArea, sweep_bar)

        # Central: images row + frequency selector + spectrum + status row
        central = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(central)

        images = QtWidgets.QHBoxLayout()
        ref_box = QtWidgets.QVBoxLayout()
        ref_box.addWidget(QtWidgets.QLabel('<b>MW OFF (reference)</b>'))
        self.image_reference = ImageWidget()
        self.image_reference.image_item.setOpts(False, axisOrder='row-major')
        ref_box.addWidget(self.image_reference)
        con_box = QtWidgets.QVBoxLayout()
        self.contrast_title = QtWidgets.QLabel('<b>(MW ON − OFF) / OFF — contrast</b>')
        con_box.addWidget(self.contrast_title)
        self.image_contrast = ImageWidget()
        self.image_contrast.image_item.setOpts(False, axisOrder='row-major')
        con_box.addWidget(self.image_contrast)
        images.addLayout(ref_box)
        images.addLayout(con_box)
        vbox.addLayout(images, stretch=3)

        # Frequency selector for the contrast panel (M4)
        freq_row = QtWidgets.QHBoxLayout()
        freq_row.addWidget(QtWidgets.QLabel('contrast panel frequency: '))
        self.freq_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.freq_slider.setRange(0, 0)
        self.freq_slider.setEnabled(False)
        freq_row.addWidget(self.freq_slider, stretch=1)
        self.freq_label = QtWidgets.QLabel('— (run a sweep)')
        freq_row.addWidget(self.freq_label)
        self.follow_checkbox = QtWidgets.QCheckBox('follow latest point')
        self.follow_checkbox.setChecked(True)
        freq_row.addWidget(self.follow_checkbox)
        vbox.addLayout(freq_row)

        # Pixel spectrum (M4)
        self.spectrum_plot = pg.PlotWidget()
        self.spectrum_plot.setLabel('bottom', 'frequency', units='Hz')
        self.spectrum_plot.setLabel('left', 'contrast')
        self.spectrum_curve = self.spectrum_plot.plot(pen=pg.mkPen(width=2),
                                                      symbol='o', symbolSize=4)
        self.spectrum_marker = pg.InfiniteLine(angle=90, movable=False,
                                               pen=pg.mkPen(style=QtCore.Qt.PenStyle.DashLine))
        self.spectrum_plot.addItem(self.spectrum_marker)
        vbox.addWidget(self.spectrum_plot, stretch=2)

        bottom = QtWidgets.QHBoxLayout()
        self.contrast_label = QtWidgets.QLabel('mean contrast: —')
        font = self.contrast_label.font()
        font.setPointSize(14)
        font.setBold(True)
        self.contrast_label.setFont(font)
        bottom.addWidget(self.contrast_label)
        self.pixel_label = QtWidgets.QLabel('pixel: — (click an image)')
        bottom.addWidget(self.pixel_label)
        bottom.addStretch()
        self.progress_label = QtWidgets.QLabel('idle')
        bottom.addWidget(self.progress_label)
        vbox.addLayout(bottom)
        self.setCentralWidget(central)


class WidefieldOdmrGui(GuiBase):
    """ Widefield-ODMR GUI (M3 burst mode + M4 sweep/cursor/selector).

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
        self._pixel = None      # (x, y) selected by cursor

    def on_activate(self):
        logic = self._widefield_logic()
        self._mw = WidefieldOdmrMainWindow()
        self._mw.pairs_spinbox.setValue(logic.default_n_pairs)

        self._mw.action_stop.setEnabled(False)
        self._mw.action_start.triggered.connect(self._start_clicked)
        self._mw.action_start_sweep.triggered.connect(self._start_sweep_clicked)
        # DIRECT connection: the logic thread is busy while measuring; stop_burst only
        # sets a flag and is safe from the GUI thread.
        self._mw.action_stop.triggered.connect(logic.stop_burst,
                                               QtCore.Qt.DirectConnection)

        logic.sigImagesUpdated.connect(self._update_images)
        logic.sigPairDone.connect(self._update_progress)
        logic.sigBurstFinished.connect(self._run_finished)
        logic.sigSweepPointDone.connect(self._sweep_point_done)
        logic.sigSweepFinished.connect(self._run_finished)

        self._mw.freq_slider.valueChanged.connect(self._freq_selected)
        # Grabbing the slider takes control back from the sweep's auto-follow.
        self._mw.freq_slider.sliderPressed.connect(
            lambda: self._mw.follow_checkbox.setChecked(False))

        # Pixel CURSOR (operator request): movable crosshair on the contrast panel;
        # its intersection is the selected pixel. Clicks on either image move it too.
        vb = self._mw.image_contrast.image_item.getViewBox()
        pen = pg.mkPen('c', width=1)
        # Free 2D drag: a TargetItem carries the position; the crosshair lines are
        # display-only and follow it.
        self._target = pg.TargetItem(pos=(600, 600), size=30, movable=True,
                                     pen=pg.mkPen('c', width=2))
        self._vline = pg.InfiniteLine(angle=90, movable=False, pen=pen)
        self._hline = pg.InfiniteLine(angle=0, movable=False, pen=pen)
        vb.addItem(self._vline)
        vb.addItem(self._hline)
        vb.addItem(self._target)
        self._target.sigPositionChanged.connect(self._cursor_moved)
        self._cursor_moved()  # attach the lines to the target's start position
        for widget in (self._mw.image_reference, self._mw.image_contrast):
            widget.image_item.scene().sigMouseClicked.connect(
                lambda ev, w=widget: self._image_clicked(ev, w))
        self.show()

    def on_deactivate(self):
        logic = self._widefield_logic()
        logic.sigSweepFinished.disconnect(self._run_finished)
        logic.sigSweepPointDone.disconnect(self._sweep_point_done)
        logic.sigBurstFinished.disconnect(self._run_finished)
        logic.sigPairDone.disconnect(self._update_progress)
        logic.sigImagesUpdated.disconnect(self._update_images)
        self._mw.action_stop.triggered.disconnect()
        self._mw.action_start_sweep.triggered.disconnect()
        self._mw.action_start.triggered.disconnect()
        self._mw.close()

    def show(self):
        self._mw.show()
        self._mw.raise_()
        self._mw.activateWindow()

    # ---------------- run control ----------------

    def _set_running(self, running):
        self._mw.action_start.setEnabled(not running)
        self._mw.action_start_sweep.setEnabled(not running)
        self._mw.action_stop.setEnabled(running)

    def _start_clicked(self):
        logic = self._widefield_logic()
        logic.set_exposure(self._mw.exposure_spinbox.value() / 1000.0)
        logic.set_period(self._mw.period_spinbox.value())
        self._set_running(True)
        self._mw.progress_label.setText('starting burst…')
        logic.start_burst(self._mw.pairs_spinbox.value())

    def _start_sweep_clicked(self):
        logic = self._widefield_logic()
        logic.set_exposure(self._mw.exposure_spinbox.value() / 1000.0)
        logic.set_period(self._mw.period_spinbox.value())
        n_points = self._mw.points_spinbox.value()
        self._mw.freq_slider.setRange(0, n_points - 1)
        self._mw.freq_slider.setEnabled(True)
        self._set_running(True)
        self._mw.progress_label.setText('starting sweep…')
        logic.start_sweep(self._mw.fstart_spinbox.value() * 1e9,
                          self._mw.fstop_spinbox.value() * 1e9,
                          n_points,
                          self._mw.ppp_spinbox.value())

    def _run_finished(self, ok, message):
        self._set_running(False)
        self._mw.progress_label.setText(('done: ' if ok else 'FAILED: ') + message)

    def _update_progress(self, done, total):
        self._mw.progress_label.setText(f'pair {done}/{total}')

    # ---------------- burst (M3) display ----------------

    def _update_images(self, reference_image, contrast_image, mean_contrast):
        self._mw.image_reference.set_image(reference_image)
        self._mw.image_contrast.set_image(contrast_image)
        self._mw.contrast_label.setText(f'mean contrast: {mean_contrast * 100:+.3f} %')
        self._mw.contrast_title.setText('<b>(MW ON − OFF) / OFF — contrast (burst)</b>')

    # ---------------- sweep (M4) display ----------------

    def _sweep_point_done(self, done, total):
        logic = self._widefield_logic()
        self._mw.progress_label.setText(f'sweep point {done}/{total}')
        if logic.sweep_reference is not None:
            self._mw.image_reference.set_image(logic.sweep_reference)
        if self._pixel is None:
            # Nothing selected yet: start the target at the image center
            # (moves the crosshair AND sets _pixel via _cursor_moved).
            ref = logic.sweep_reference
            if ref is not None:
                self._target.setPos(ref.shape[1] // 2, ref.shape[0] // 2)
        if self._mw.follow_checkbox.isChecked():
            # Moves the slider -> triggers _freq_selected -> updates the panel.
            self._mw.freq_slider.setValue(done - 1)
        self._update_spectrum()

    def _freq_selected(self, idx):
        logic = self._widefield_logic()
        freqs = logic.sweep_frequencies
        stack = logic.contrast_stack
        if freqs is None or stack is None or not (0 <= idx < len(freqs)):
            return
        self._mw.freq_label.setText(f'{freqs[idx] / 1e9:.4f} GHz [{idx + 1}/{len(freqs)}]')
        frame = stack[idx]
        if not np.all(np.isnan(frame)):
            self._mw.image_contrast.set_image(frame)
            self._mw.contrast_title.setText(
                f'<b>contrast @ {freqs[idx] / 1e9:.4f} GHz</b>')
            mean_c = float(np.nanmean(frame))
            self._mw.contrast_label.setText(f'mean contrast: {mean_c * 100:+.3f} %')
        self._mw.spectrum_marker.setValue(freqs[idx])

    def _image_clicked(self, ev, widget):
        try:
            pos = widget.image_item.mapFromScene(ev.scenePos())
            # Move the target; _cursor_moved does the rest.
            self._target.setPos(pos.x(), pos.y())
        except Exception:
            self.log.exception('Image click handling failed:')

    def _cursor_moved(self):
        pos = self._target.pos()
        self._vline.setValue(pos.x())
        self._hline.setValue(pos.y())
        img = self._mw.image_contrast.image_item.image
        if img is None:
            return
        x = int(np.clip(pos.x(), 0, img.shape[1] - 1))
        y = int(np.clip(pos.y(), 0, img.shape[0] - 1))
        self._pixel = (x, y)
        self._mw.pixel_label.setText(f'pixel: ({x}, {y})')
        self._update_spectrum()

    def _update_spectrum(self):
        try:
            logic = self._widefield_logic()
            freqs = logic.sweep_frequencies
            stack = logic.contrast_stack
            if freqs is None or stack is None:
                # The spectrum is SWEEP data — a fixed-frequency burst cannot fill it.
                self._mw.spectrum_plot.setTitle('no sweep data — use Start Sweep')
                return
            if self._pixel is None:
                self._mw.spectrum_plot.setTitle('no pixel selected — click the image')
                return
            x, y = self._pixel
            if not (0 <= y < stack.shape[1] and 0 <= x < stack.shape[2]):
                self._mw.spectrum_plot.setTitle(f'pixel ({x},{y}) out of bounds')
                return
            spectrum = np.asarray(stack[:, y, x], dtype=float)
            valid = ~np.isnan(spectrum)
            n_valid = int(np.count_nonzero(valid))
            self._mw.spectrum_plot.setTitle(
                f'pixel ({x},{y}) — {n_valid}/{len(freqs)} points')
            if n_valid:
                self._mw.spectrum_curve.setData(np.asarray(freqs, dtype=float)[valid],
                                                spectrum[valid])
                self._mw.spectrum_plot.enableAutoRange()
        except Exception:
            self._mw.spectrum_plot.setTitle('spectrum ERROR — see qudi log/console')
            self.log.exception('Spectrum update failed:')
