# SPDX-License-Identifier: AGPL-3.0-or-later
from __future__ import annotations

import json
import sys
import time
from collections import deque
from pathlib import Path
from typing import Deque, Dict, List, Optional

from .core import (
    DOPPLER_HZ_TO_KMH,
    FFT_DBM_DISPLAY_MAX_DBM,
    FFT_DBM_DISPLAY_MIN_DBM,
    FFT_DB_DISPLAY_TOP_DB,
    FFT_Y_AXIS_LIMIT_TOP_DB,
    RADAR_CARRIER_HZ,
    SPECTROGRAM_FLOOR_DB,
    SPECTROGRAM_TOP_DB,
    WATERFALL_COMPRESSION_TAU_DB,
    WATERFALL_DISPLAY_TOP_DB,
    AnalysisSnapshot,
    RadarAnalyzer,
    Telemetry,
    format_duration,
    frame_to_centered_iq,
    frames_to_centered_iq,
    timestamp_for_filename,
)

STATUS_REFRESH_S = 0.5

class LivePlotter:
    def __init__(self, window_samples: int, fft_size: int, refresh_hz: float) -> None:
        import matplotlib.pyplot as plt
        import numpy as np

        self._np = np
        self._plt = plt
        self.window_samples = max(1024, window_samples)
        self.fft_size = max(256, min(fft_size, self.window_samples))
        self._analyzer = RadarAnalyzer(np, self.fft_size)
        self.refresh_period = 1.0 / max(1.0, refresh_hz)
        self.analysis_period = self.refresh_period
        self.last_draw = 0.0
        self._last_analysis_at = 0.0
        self.sample_rate_hz = 1

        self.i_buffer: Deque[float] = deque(maxlen=self.window_samples)
        self.q_buffer: Deque[float] = deque(maxlen=self.window_samples)
        self.spectrogram_columns = 128
        self.spectrogram = np.full((self.fft_size, self.spectrogram_columns), SPECTROGRAM_FLOOR_DB, dtype=np.float32)

        self.figure, axes = plt.subplots(3, 2, figsize=(14, 10))
        self.ax_time = axes[0][0]
        self.ax_mag = axes[0][1]
        self.ax_const = axes[1][0]
        self.ax_phase = axes[1][1]
        self.ax_fft = axes[2][0]
        self.ax_spec = axes[2][1]

        self.time_i_line, = self.ax_time.plot([], [], label="I", linewidth=1.2)
        self.time_q_line, = self.ax_time.plot([], [], label="Q", linewidth=1.2)
        self.mag_line, = self.ax_mag.plot([], [], label="|IQ|", linewidth=1.2)
        self.const_points = self.ax_const.scatter([], [], s=6, alpha=0.45)
        self.phase_line, = self.ax_phase.plot([], [], linewidth=1.2)
        self.fft_line, = self.ax_fft.plot([], [], linewidth=1.2)
        self.fft_peak_line = self.ax_fft.axvline(0.0, color="#c62828", linewidth=1.0, alpha=0.8)
        self.spec_image = self.ax_spec.imshow(
            self.spectrogram,
            origin="lower",
            aspect="auto",
            cmap="cividis",
            vmin=SPECTROGRAM_FLOOR_DB,
            vmax=SPECTROGRAM_TOP_DB,
            interpolation="nearest",
        )
        self.spec_peak_line = self.ax_spec.axhline(0.0, color="#c62828", linewidth=1.0, alpha=0.8)

        self.ax_time.set_title("I/Q Time Domain")
        self.ax_time.legend(loc="upper right")
        self.ax_mag.set_title("Magnitude Envelope")
        self.ax_const.set_title("IQ Constellation")
        self.ax_phase.set_title("Unwrapped Phase")
        self.ax_fft.set_title("Signed Doppler Spectrum")
        self.ax_spec.set_title("Doppler Waterfall")

        for axis in (self.ax_time, self.ax_mag, self.ax_const, self.ax_phase, self.ax_fft):
            axis.grid(True, alpha=0.2)

        self.ax_time.set_xlabel("Sample")
        self.ax_time.set_ylabel("Centered code")
        self.ax_mag.set_xlabel("Sample")
        self.ax_mag.set_ylabel("Magnitude")
        self.ax_const.set_xlabel("I")
        self.ax_const.set_ylabel("Q")
        self.ax_phase.set_xlabel("Sample")
        self.ax_phase.set_ylabel("Radians")
        self.ax_fft.set_xlabel(f"Doppler frequency (Hz), 1 Hz = {DOPPLER_HZ_TO_KMH:0.5f} km/h")
        self.ax_fft.set_ylabel("Excess over background (dB)")
        self.ax_spec.set_xlabel("Time history")
        self.ax_spec.set_ylabel("Doppler frequency (Hz)")

        plt.tight_layout()
        plt.ion()
        plt.show(block=False)

    def update(self, frame: RadarFrame) -> None:
        i, q = frame_to_centered_iq(frame, self._np)
        self.i_buffer.extend(i)
        self.q_buffer.extend(q)
        self.sample_rate_hz = max(1, frame.sample_rate_hz)

    def update_batch(self, frames) -> None:
        if not frames:
            return
        i, q, sample_rate_hz = frames_to_centered_iq(frames, self._np)
        if i.size == 0:
            return
        self.i_buffer.extend(i)
        self.q_buffer.extend(q)
        self.sample_rate_hz = max(1, sample_rate_hz)

    def update_telemetry(self, telemetry: Telemetry, started_at: float) -> None:
        _ = telemetry
        _ = started_at

    def is_active(self) -> bool:
        return True

    def draw(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self.last_draw) < self.refresh_period:
            return
        if len(self.i_buffer) < 8:
            return

        np = self._np
        analysis_due = force or (now - self._last_analysis_at) >= self.analysis_period
        snapshot = getattr(self, "_last_snapshot", None)
        if analysis_due or snapshot is None:
            snapshot = self._analyzer.analyze(self.i_buffer, self.q_buffer, self.sample_rate_hz, self.window_samples)
            if snapshot is None:
                return
            self._last_snapshot = snapshot
            self._last_analysis_at = now

        self.time_i_line.set_data(snapshot.sample_axis, snapshot.i)
        self.time_q_line.set_data(snapshot.sample_axis, snapshot.q)
        self.ax_time.set_xlim(0, max(1, len(snapshot.i) - 1))
        span = max(16.0, float(max(snapshot.i.max(), snapshot.q.max()) - min(snapshot.i.min(), snapshot.q.min())))
        center = float(0.5 * (max(snapshot.i.max(), snapshot.q.max()) + min(snapshot.i.min(), snapshot.q.min())))
        self.ax_time.set_ylim(center - 0.6 * span, center + 0.6 * span)

        self.mag_line.set_data(snapshot.sample_axis, snapshot.magnitude)
        self.ax_mag.set_xlim(0, max(1, snapshot.magnitude.size - 1))
        self.ax_mag.set_ylim(0.0, max(32.0, float(snapshot.magnitude.max()) * 1.1))

        stride = max(1, len(snapshot.i) // 2500)
        const_points = np.column_stack((snapshot.i[::stride], snapshot.q[::stride]))
        self.const_points.set_offsets(const_points)
        const_limit = max(32.0, float(np.max(np.abs(const_points))) * 1.15)
        self.ax_const.set_xlim(-const_limit, const_limit)
        self.ax_const.set_ylim(-const_limit, const_limit)

        self.phase_line.set_data(snapshot.sample_axis, snapshot.phase)
        self.ax_phase.set_xlim(0, max(1, snapshot.phase.size - 1))
        phase_span = max(1.0, float(snapshot.phase.max() - snapshot.phase.min()))
        self.ax_phase.set_ylim(float(snapshot.phase.min()) - 0.1 * phase_span, float(snapshot.phase.max()) + 0.1 * phase_span)

        if snapshot.freq_axis is not None and snapshot.fft_magnitude is not None:
            self.fft_line.set_data(snapshot.freq_axis, snapshot.fft_magnitude)
            self.ax_fft.set_xlim(float(snapshot.freq_axis[0]), float(snapshot.freq_axis[-1]))
            self.ax_fft.set_ylim(SPECTROGRAM_FLOOR_DB, SPECTROGRAM_TOP_DB)
            if snapshot.detector_state == "locked":
                self.fft_peak_line.set_xdata([snapshot.dominant_hz, snapshot.dominant_hz])
                self.spec_peak_line.set_ydata([snapshot.dominant_hz, snapshot.dominant_hz])

            self.spectrogram = np.roll(self.spectrogram, -1, axis=1)
            self.spectrogram[:, -1] = snapshot.fft_magnitude.astype(np.float32)
            self.spec_image.set_data(self.spectrogram)
            self.spec_image.set_clim(SPECTROGRAM_FLOOR_DB, SPECTROGRAM_TOP_DB)
            self.spec_image.set_extent((-self.spectrogram.shape[1], 0.0, float(snapshot.freq_axis[0]), float(snapshot.freq_axis[-1])))
            self.ax_spec.set_xlim(-self.spectrogram.shape[1], 0.0)
            self.ax_spec.set_ylim(float(snapshot.freq_axis[0]), float(snapshot.freq_axis[-1]))

        self.figure.canvas.draw_idle()
        self._plt.pause(0.001)
        self.last_draw = now

    def close(self) -> None:
        self.draw(force=True)
        self._plt.ioff()
        self._plt.close(self.figure)

    def process_events(self) -> None:
        self._plt.pause(0.001)

class PyQtGraphRadarStudio:
    def __init__(
        self,
        window_samples: int,
        fft_size: int,
        refresh_hz: float,
        source_name: str = "serial",
        recording_enabled: bool = False,
    ) -> None:
        import numpy as np

        try:
            from PySide6 import QtCore, QtGui, QtWidgets
            horizontal_orientation = QtCore.Qt.Orientation.Horizontal
            dash_line = QtCore.Qt.PenStyle.DashLine
        except ImportError:
            try:
                from PyQt6 import QtCore, QtGui, QtWidgets
                horizontal_orientation = QtCore.Qt.Orientation.Horizontal
                dash_line = QtCore.Qt.PenStyle.DashLine
            except ImportError:
                try:
                    from PyQt5 import QtCore, QtGui, QtWidgets
                    horizontal_orientation = QtCore.Qt.Horizontal
                    dash_line = QtCore.Qt.DashLine
                except ImportError as exc:
                    raise ImportError(
                        "Qt-Bindings fehlen. Installiere z.B. `python3 -m pip install PySide6 pyqtgraph`."
                    ) from exc

        try:
            import pyqtgraph as pg
        except ImportError as exc:
            raise ImportError("pyqtgraph fehlt. Installiere z.B. `python3 -m pip install pyqtgraph`.") from exc

        self._np = np
        self._QtCore = QtCore
        self._QShortcut = QtGui.QShortcut if hasattr(QtGui, "QShortcut") else QtWidgets.QShortcut
        self._QtWidgets = QtWidgets
        self._pg = pg
        self._dash_line = dash_line
        self.window_samples = max(32768, window_samples)
        self.fft_size = max(512, min(fft_size, self.window_samples))
        self._analyzer = RadarAnalyzer(np, self.fft_size)
        self.refresh_period = 1.0 / max(1.0, refresh_hz)
        self.analysis_period = self.refresh_period
        self.last_draw = 0.0
        self._last_analysis_at = 0.0
        self._last_status_update = 0.0
        self.sample_rate_hz = 1
        self.started_at = time.monotonic()
        self._telemetry = Telemetry()
        self._closed = False
        self._paused = False
        self._display_window = self.window_samples
        self._current_fft_size = self.fft_size
        self._fft_display_top_db = FFT_DB_DISPLAY_TOP_DB
        self._active_spectrum_lower = SPECTROGRAM_FLOOR_DB
        self._active_spectrum_upper = FFT_DB_DISPLAY_TOP_DB
        self._spectrum_scale_mode = "db"
        self._spectrogram_columns = 360
        self._spectrogram_initialized = False
        self._last_snapshot: Optional[AnalysisSnapshot] = None
        self._target_events: List[dict] = []
        self._active_target_events: List[dict] = []
        self._next_target_event_id = 1
        self._last_target_log_render = 0.0
        self._last_target_log_text = ""
        self._secondary_fft_lines = []
        self._secondary_spec_lines = []
        self._limits_sample_rate_hz = 0
        self._spectrogram_rect_dirty = True
        self._last_card_texts: Dict[str, str] = {}
        self._last_quality_badge = ""
        self._last_lock_badge = ""
        self._last_status_message = ""
        self._last_autotune_generation = -1
        self.source_name = source_name
        self.recording_enabled = recording_enabled

        self.i_buffer: Deque[float] = deque(maxlen=self.window_samples)
        self.q_buffer: Deque[float] = deque(maxlen=self.window_samples)
        self._configure_spectrogram_buffers()

        pg.setConfigOptions(antialias=False, background="#f5f8fb", foreground="#1b2a38", imageAxisOrder="row-major")
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.app.setApplicationName("Radar Capture")

        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle("Radar Capture")
        self.window.resize(1760, 1200)

        def handle_close(event) -> None:
            self._closed = True
            event.accept()

        self.window.closeEvent = handle_close  # type: ignore[assignment]

        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(12)

        header = QtWidgets.QFrame()
        header.setProperty("hero", True)
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(20, 16, 20, 16)
        title_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("Radar Capture")
        title_font = QtGui.QFont()
        title_font.setPointSize(20)
        title_font.setBold(True)
        title.setFont(title_font)
        subtitle = QtWidgets.QLabel(f"Live Doppler and I/Q monitoring on {self.source_name}.")
        subtitle.setStyleSheet("color: #607488;")
        subtitle.setWordWrap(True)
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header_layout.addLayout(title_box, stretch=1)
        self.mode_badge = self._badge("LIVE")
        self.lock_badge = self._badge("SEARCHING")
        self.quality_badge = self._badge("QUALITY OK")
        self.source_badge = self._badge(f"SOURCE {self.source_name}")
        header_layout.addWidget(self.mode_badge)
        header_layout.addWidget(self.lock_badge)
        header_layout.addWidget(self.quality_badge)
        header_layout.addWidget(self.source_badge)
        root.addWidget(header)

        splitter = QtWidgets.QSplitter(horizontal_orientation)
        splitter.setChildrenCollapsible(False)
        root.addWidget(splitter, stretch=1)

        side = QtWidgets.QWidget()
        side_layout = QtWidgets.QVBoxLayout(side)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(12)
        side_scroll = QtWidgets.QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        side_scroll.setWidget(side)
        splitter.addWidget(side_scroll)

        cards = QtWidgets.QFrame()
        cards.setProperty("panel", True)
        cards_layout = QtWidgets.QGridLayout(cards)
        cards_layout.setContentsMargins(12, 12, 12, 12)
        cards_layout.setHorizontalSpacing(8)
        cards_layout.setVerticalSpacing(8)
        self._card_values: Dict[str, object] = {}
        card_defs = [
            ("fps", "Frames/s"),
            ("sample_rate", "Sample Rate"),
            ("scene", "Scene"),
            ("dominant_hz", "Dominant Hz"),
            ("velocity", "Velocity"),
            ("direction", "Direction"),
            ("snr", "Peak SNR"),
            ("detector", "Detector"),
            ("targets", "Targets"),
            ("rms", "RMS |IQ|"),
            ("phase_std", "Phase Std"),
            ("motion_index", "Motion Index"),
            ("spectral_spread", "Spectral Spread"),
            ("coherence", "Coherence"),
            ("device_drops", "Device Drops"),
            ("crc", "CRC Errors"),
            ("recording", "Recording"),
        ]
        for index, (key, label) in enumerate(card_defs):
            cards_layout.addWidget(self._stat_card(label, key), index // 4, index % 4)
        side_layout.addWidget(cards)

        controls = QtWidgets.QFrame()
        controls.setProperty("panel", True)
        controls_layout = QtWidgets.QVBoxLayout(controls)
        controls_layout.setContentsMargins(14, 14, 14, 14)
        controls_layout.setSpacing(10)
        section = QtWidgets.QLabel("Analysis Controls")
        section.setProperty("sectionTitle", True)
        controls_layout.addWidget(section)
        form = QtWidgets.QFormLayout()
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(10)

        self.window_combo = QtWidgets.QComboBox()
        for value in ("2048", "4096", "8192", "16384", "32768"):
            self.window_combo.addItem(value)
        self.window_combo.setCurrentText(str(self._display_window))
        self.window_combo.currentTextChanged.connect(self._on_window_changed)
        form.addRow("Display Window", self.window_combo)

        self.fft_combo = QtWidgets.QComboBox()
        for value in ("512", "1024", "2048", "4096", "8192", "16384"):
            self.fft_combo.addItem(value)
        self.fft_combo.setCurrentText(str(self._current_fft_size))
        self.fft_combo.currentTextChanged.connect(self._on_fft_changed)
        form.addRow("FFT Size", self.fft_combo)

        self.doppler_span_combo = QtWidgets.QComboBox()
        for label, value in (
            ("+/- 250 Hz", "250"),
            ("+/- 500 Hz", "500"),
            ("+/- 1 kHz", "1000"),
            ("+/- 2.5 kHz", "2500"),
            ("+/- 5 kHz", "5000"),
            ("Full Nyquist", "full"),
        ):
            self.doppler_span_combo.addItem(label, value)
        self.doppler_span_combo.setCurrentText("+/- 2.5 kHz")
        self.doppler_span_combo.currentIndexChanged.connect(self._on_doppler_span_changed)
        form.addRow("Doppler Zoom", self.doppler_span_combo)

        self.direction_combo = QtWidgets.QComboBox()
        self.direction_combo.addItem("Positive Doppler = approaching", 1.0)
        self.direction_combo.addItem("Positive Doppler = receding", -1.0)
        self.direction_combo.currentIndexChanged.connect(self._on_direction_changed)
        form.addRow("I/Q Direction", self.direction_combo)

        self.notch_combo = QtWidgets.QComboBox()
        for label, value in (("Off", "0"), ("+/- 25 Hz", "25"), ("+/- 50 Hz", "50"), ("+/- 100 Hz", "100"), ("+/- 250 Hz", "250")):
            self.notch_combo.addItem(label, value)
        self.notch_combo.setCurrentText("+/- 50 Hz")
        self.notch_combo.currentIndexChanged.connect(self._on_notch_changed)
        form.addRow("Clutter Notch", self.notch_combo)

        self.y_scale_combo = QtWidgets.QComboBox()
        self.y_scale_combo.addItem("0 .. 50 dB", "db")
        self.y_scale_combo.addItem("0 .. 100 dBm", "dbm")
        self.y_scale_combo.setCurrentIndex(0)
        self.y_scale_combo.currentIndexChanged.connect(self._on_y_scale_changed)
        form.addRow("Y Scale", self.y_scale_combo)

        self.dc_checkbox = QtWidgets.QCheckBox("Remove DC before FFT and phase analysis")
        self.dc_checkbox.setChecked(True)
        self.dc_checkbox.toggled.connect(self._on_remove_dc_toggled)
        form.addRow("Conditioning", self.dc_checkbox)

        self.multi_target_checkbox = QtWidgets.QCheckBox("Enable multi-target detection")
        self.multi_target_checkbox.setChecked(False)
        self.multi_target_checkbox.toggled.connect(self._on_multi_target_toggled)
        form.addRow("Detection", self.multi_target_checkbox)

        self.max_targets_combo = QtWidgets.QComboBox()
        for value in ("2", "3", "4", "5"):
            self.max_targets_combo.addItem(value)
        self.max_targets_combo.setCurrentText("3")
        self.max_targets_combo.currentTextChanged.connect(self._on_max_targets_changed)
        form.addRow("Max Targets", self.max_targets_combo)

        self.acquire_snr_spin = QtWidgets.QDoubleSpinBox()
        self.acquire_snr_spin.setRange(0.0, 60.0)
        self.acquire_snr_spin.setSingleStep(1.0)
        self.acquire_snr_spin.setDecimals(1)
        self.acquire_snr_spin.setValue(self._analyzer.detector_acquire_snr_db)
        self.acquire_snr_spin.valueChanged.connect(self._on_acquire_snr_changed)
        form.addRow("Acquire SNR", self.acquire_snr_spin)

        self.release_snr_spin = QtWidgets.QDoubleSpinBox()
        self.release_snr_spin.setRange(0.0, 60.0)
        self.release_snr_spin.setSingleStep(1.0)
        self.release_snr_spin.setDecimals(1)
        self.release_snr_spin.setValue(self._analyzer.detector_release_snr_db)
        self.release_snr_spin.valueChanged.connect(self._on_release_snr_changed)
        form.addRow("Release SNR", self.release_snr_spin)

        self.noise_guard_combo = QtWidgets.QComboBox()
        self.noise_guard_combo.addItem("Adaptive", "adaptive")
        self.noise_guard_combo.addItem("Conservative", "conservative")
        self.noise_guard_combo.currentIndexChanged.connect(self._on_noise_guard_mode_changed)
        form.addRow("Noise Guard", self.noise_guard_combo)

        self.noise_avg_margin_spin = QtWidgets.QDoubleSpinBox()
        self.noise_avg_margin_spin.setRange(0.0, 40.0)
        self.noise_avg_margin_spin.setSingleStep(0.5)
        self.noise_avg_margin_spin.setDecimals(1)
        self.noise_avg_margin_spin.setValue(self._analyzer.noise_guard_avg_margin_db)
        self.noise_avg_margin_spin.valueChanged.connect(self._on_noise_avg_margin_changed)
        form.addRow("Avg Margin", self.noise_avg_margin_spin)

        self.noise_max_margin_spin = QtWidgets.QDoubleSpinBox()
        self.noise_max_margin_spin.setRange(0.0, 20.0)
        self.noise_max_margin_spin.setSingleStep(0.5)
        self.noise_max_margin_spin.setDecimals(1)
        self.noise_max_margin_spin.setValue(self._analyzer.noise_guard_max_margin_db)
        self.noise_max_margin_spin.valueChanged.connect(self._on_noise_max_margin_changed)
        form.addRow("Max Margin", self.noise_max_margin_spin)
        controls_layout.addLayout(form)

        actions = QtWidgets.QHBoxLayout()
        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self._toggle_pause)
        self.reset_button = QtWidgets.QPushButton("Reset Buffers")
        self.reset_button.clicked.connect(self._reset_buffers)
        self.snapshot_button = QtWidgets.QPushButton("Save Snapshot")
        self.snapshot_button.clicked.connect(self._save_snapshot)
        self.export_button = QtWidgets.QPushButton("Export Metrics")
        self.export_button.clicked.connect(self._export_metrics)
        self.reset_view_button = QtWidgets.QPushButton("Reset View")
        self.reset_view_button.clicked.connect(self._reset_view_ranges)
        actions.addWidget(self.pause_button)
        actions.addWidget(self.reset_button)
        actions.addWidget(self.reset_view_button)
        actions.addWidget(self.snapshot_button)
        actions.addWidget(self.export_button)
        controls_layout.addLayout(actions)
        side_layout.addWidget(controls)

        target_log_panel = QtWidgets.QFrame()
        target_log_panel.setProperty("panel", True)
        target_log_layout = QtWidgets.QVBoxLayout(target_log_panel)
        target_log_layout.setContentsMargins(14, 14, 14, 14)
        target_log_layout.setSpacing(8)
        target_log_title = QtWidgets.QLabel("Target Log")
        target_log_title.setProperty("sectionTitle", True)
        target_log_layout.addWidget(target_log_title)
        target_log_actions = QtWidgets.QHBoxLayout()
        target_log_actions.addStretch(1)
        self.target_log_clear_button = QtWidgets.QPushButton("Clear")
        self.target_log_clear_button.clicked.connect(self._clear_target_log)
        target_log_actions.addWidget(self.target_log_clear_button)
        target_log_layout.addLayout(target_log_actions)
        self.target_log = QtWidgets.QPlainTextEdit()
        self.target_log.setReadOnly(True)
        self.target_log.setMinimumHeight(140)
        self.target_log.setPlainText("No target history yet.")
        target_log_layout.addWidget(self.target_log)
        side_layout.addWidget(target_log_panel)
        side_layout.addStretch(1)
        side_layout.addStretch(1)

        plots = QtWidgets.QFrame()
        plots.setProperty("panel", True)
        plot_layout = pg.GraphicsLayoutWidget()
        plot_layout.setMinimumHeight(920)
        plot_wrap = QtWidgets.QVBoxLayout(plots)
        plot_wrap.setContentsMargins(10, 10, 10, 20)
        plot_wrap.setSpacing(8)
        plot_wrap.addWidget(plot_layout)
        splitter.addWidget(plots)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([610, 1150])

        self.spec_plot = plot_layout.addPlot(row=0, col=0, colspan=2, title="Doppler Waterfall")
        self.spec_plot.setLabel("left", "Doppler", units="Hz")
        self.spec_plot.setLabel("bottom", "History")
        self.spec_plot.showGrid(x=True, y=True, alpha=0.18)
        self.spec_image = pg.ImageItem()
        self.spec_image.setLookupTable(self._waterfall_lut())
        if hasattr(self.spec_image, "setAutoDownsample"):
            self.spec_image.setAutoDownsample(True)
        self.spec_plot.addItem(self.spec_image)
        self.spec_peak_line = pg.InfiniteLine(angle=0, pen=pg.mkPen("#c62828", width=1))
        self.spec_zero_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#455a64", width=1))
        self.spec_plot.addItem(self.spec_zero_line)
        self.spec_plot.addItem(self.spec_peak_line)
        self._spec_viewbox = self.spec_plot.getViewBox()
        self.spec_plot.setMouseEnabled(x=False, y=True)
        for _ in range(4):
            line = pg.InfiniteLine(angle=0, pen=pg.mkPen("#ef6c00", width=1, style=self._dash_line))
            line.hide()
            self.spec_plot.addItem(line)
            self._secondary_spec_lines.append(line)

        self.fft_plot = plot_layout.addPlot(row=1, col=0, colspan=2, title="Doppler Spectrum")
        self.fft_plot.setLabel("bottom", f"Doppler, 1 Hz = {DOPPLER_HZ_TO_KMH:0.5f} km/h", units="Hz")
        self.fft_plot.setLabel("left", "Excess over background", units="dB")
        self.fft_plot.setYRange(SPECTROGRAM_FLOOR_DB, self._fft_display_top_db, padding=0.0)
        self.fft_plot.showGrid(x=True, y=True, alpha=0.18)
        self.fft_curve = self.fft_plot.plot(pen=pg.mkPen("#ef6c00", width=2))
        self.fft_plot.setDownsampling(auto=True, mode="peak")
        self.fft_plot.setClipToView(True)
        self.fft_peak_line = pg.InfiniteLine(angle=90, pen=pg.mkPen("#c62828", width=1))
        self.fft_zero_line = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen("#455a64", width=1))
        self.fft_plot.addItem(self.fft_zero_line)
        self.fft_plot.addItem(self.fft_peak_line)
        self._fft_viewbox = self.fft_plot.getViewBox()
        for _ in range(4):
            line = pg.InfiniteLine(angle=90, pen=pg.mkPen("#ef6c00", width=1, style=self._dash_line))
            line.hide()
            self.fft_plot.addItem(line)
            self._secondary_fft_lines.append(line)

        self.cfar_plot = plot_layout.addPlot(row=2, col=0, colspan=2, title="Tracker Diagnostics")
        self.cfar_plot.setLabel("bottom", "Doppler", units="Hz")
        self.cfar_plot.setLabel("left", "Level", units="dB")
        self.cfar_plot.setYRange(SPECTROGRAM_FLOOR_DB, self._fft_display_top_db, padding=0.0)
        self.cfar_plot.showGrid(x=True, y=True, alpha=0.18)
        self.cfar_signal_curve = self.cfar_plot.plot(pen=pg.mkPen("#1565c0", width=1.8))
        self.cfar_noise_curve = self.cfar_plot.plot(pen=pg.mkPen("#607d8b", width=1.2))
        self.cfar_threshold_curve = self.cfar_plot.plot(pen=pg.mkPen("#c62828", width=1.8, style=self._dash_line))
        self.cfar_plot.setDownsampling(auto=True, mode="peak")
        self.cfar_plot.setClipToView(True)
        self.cfar_peak_line = pg.InfiniteLine(angle=90, pen=pg.mkPen("#00acc1", width=2))
        self.cfar_plot.addItem(self.cfar_peak_line)
        self.cfar_gate_left = pg.InfiniteLine(angle=90, pen=pg.mkPen("#00897b", width=1, style=self._dash_line))
        self.cfar_gate_right = pg.InfiniteLine(angle=90, pen=pg.mkPen("#00897b", width=1, style=self._dash_line))
        self.cfar_plot.addItem(self.cfar_gate_left)
        self.cfar_plot.addItem(self.cfar_gate_right)
        self._cfar_viewbox = self.cfar_plot.getViewBox()
        self.cfar_targets = pg.ScatterPlotItem(size=9, pen=pg.mkPen("#ff8f00", width=1.5), brush=pg.mkBrush(255, 143, 0, 180))
        self.cfar_plot.addItem(self.cfar_targets)
        self.cfar_primary_target = pg.ScatterPlotItem(size=12, pen=pg.mkPen("#00acc1", width=1.5), brush=pg.mkBrush(0, 172, 193, 210))
        self.cfar_plot.addItem(self.cfar_primary_target)
        self.cfar_info = pg.TextItem(anchor=(0, 0), color="#34495e", border=pg.mkPen("#d6e0ea"), fill=pg.mkBrush(255, 255, 255, 225))
        self.cfar_plot.addItem(self.cfar_info)

        self.filter_plot = plot_layout.addPlot(row=3, col=0, colspan=2, title="Adaptive Filter")
        self.filter_plot.setLabel("bottom", "Doppler", units="Hz")
        self.filter_plot.setLabel("left", "Attenuation", units="dB")
        self.filter_plot.showGrid(x=True, y=True, alpha=0.18)
        self.filter_attenuation_curve = self.filter_plot.plot(pen=pg.mkPen("#8e24aa", width=1.8))
        self.filter_spur_curve = self.filter_plot.plot(pen=pg.mkPen("#6d4c41", width=1.0, style=self._dash_line))
        self.filter_plot.setDownsampling(auto=True, mode="peak")
        self.filter_plot.setClipToView(True)
        self.filter_zero_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#455a64", width=1))
        self.filter_plot.addItem(self.filter_zero_line)
        self.filter_info = pg.TextItem(
            anchor=(0, 0),
            color="#34495e",
            border=pg.mkPen("#d6e0ea"),
            fill=pg.mkBrush(255, 255, 255, 225),
        )
        self.filter_plot.addItem(self.filter_info)
        if hasattr(plot_layout.ci.layout, "setVerticalSpacing"):
            plot_layout.ci.layout.setVerticalSpacing(8)
        if hasattr(plot_layout.ci.layout, "setRowStretchFactor"):
            plot_layout.ci.layout.setRowStretchFactor(0, 5)
            plot_layout.ci.layout.setRowStretchFactor(1, 3)
            plot_layout.ci.layout.setRowStretchFactor(2, 3)
            plot_layout.ci.layout.setRowStretchFactor(3, 3)
        self._apply_plot_limits()

        self.window.setCentralWidget(central)
        self.status_bar = QtWidgets.QStatusBar()
        self.window.setStatusBar(self.status_bar)
        self.status_bar.showMessage(f"Ready on {self.source_name}.")
        self.window.setStyleSheet(
            """
            QMainWindow, QWidget { background: #f3f7fb; color: #16202a; font-size: 13px; }
            QFrame[hero="true"] { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #ffffff, stop:1 #eef4fa); border: 1px solid #dce5ef; border-radius: 18px; }
            QFrame[panel="true"] { background: white; border: 1px solid #dce5ef; border-radius: 18px; }
            QFrame[card="true"] { background: #fbfdff; border: 1px solid #dde7f1; border-radius: 14px; }
            QLabel[cardTitle="true"] { color: #6a7b8f; font-size: 11px; letter-spacing: 0.08em; text-transform: uppercase; }
            QLabel[cardValue="true"] { color: #13202d; font-size: 14px; font-weight: 650; }
            QLabel[sectionTitle="true"] { color: #213244; font-size: 14px; font-weight: 650; }
            QLabel[badge="true"] { background: #e8f0f8; color: #1c4e80; border: 1px solid #ceddec; border-radius: 10px; padding: 6px 10px; font-weight: 650; }
            QComboBox, QPushButton, QCheckBox { border: 1px solid #d8e2ed; border-radius: 10px; background: #fcfdff; min-height: 34px; padding: 4px 10px; }
            QPushButton { background: #edf4fb; }
            QPushButton:hover { background: #e1edf9; }
            """
        )
        self.window.show()
        self._shortcuts = [
            self._QShortcut(QtGui.QKeySequence("Space"), self.window),
            self._QShortcut(QtGui.QKeySequence("R"), self.window),
            self._QShortcut(QtGui.QKeySequence("Ctrl+S"), self.window),
            self._QShortcut(QtGui.QKeySequence("Ctrl+0"), self.window),
        ]
        self._shortcuts[0].activated.connect(self._toggle_pause)
        self._shortcuts[1].activated.connect(self._reset_buffers)
        self._shortcuts[2].activated.connect(self._save_snapshot)
        self._shortcuts[3].activated.connect(self._reset_view_ranges)

    def _badge(self, text: str):
        label = self._QtWidgets.QLabel(text)
        label.setProperty("badge", True)
        return label

    def _stat_card(self, label: str, key: str):
        frame = self._QtWidgets.QFrame()
        frame.setProperty("card", True)
        frame.setMinimumWidth(150)
        layout = self._QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(10, 8, 10, 8)
        title = self._QtWidgets.QLabel(label)
        title.setProperty("cardTitle", True)
        value = self._QtWidgets.QLabel("-")
        value.setProperty("cardValue", True)
        value.setWordWrap(True)
        value.setMinimumHeight(34)
        layout.addWidget(title)
        layout.addWidget(value)
        self._card_values[key] = value
        return frame

    def _waterfall_lut(self):
        np = self._np
        stops = [
            (0.00, (2, 4, 10)),
            (0.42, (8, 18, 40)),
            (0.68, (18, 67, 144)),
            (0.86, (58, 160, 220)),
            (0.97, (245, 164, 66)),
            (1.00, (255, 248, 240)),
        ]
        table = np.zeros((256, 3), dtype=np.ubyte)
        for index in range(256):
            position = index / 255.0
            for left, right in zip(stops[:-1], stops[1:]):
                if left[0] <= position <= right[0]:
                    span = max(1e-6, right[0] - left[0])
                    mix = (position - left[0]) / span
                    color = [
                        int(round((1.0 - mix) * left[1][channel] + mix * right[1][channel]))
                        for channel in range(3)
                    ]
                    table[index] = color
                    break
        return table

    def update(self, frame: RadarFrame) -> None:
        if self._paused:
            return
        i, q = frame_to_centered_iq(frame, self._np)
        self.i_buffer.extend(i)
        self.q_buffer.extend(q)
        self.sample_rate_hz = max(1, frame.sample_rate_hz)
        if self._limits_sample_rate_hz != self.sample_rate_hz:
            self._apply_plot_limits()

    def update_batch(self, frames) -> None:
        if self._paused or not frames:
            return
        i, q, sample_rate_hz = frames_to_centered_iq(frames, self._np)
        if i.size == 0:
            return
        self.i_buffer.extend(i)
        self.q_buffer.extend(q)
        self.sample_rate_hz = max(1, sample_rate_hz)
        if self._limits_sample_rate_hz != self.sample_rate_hz:
            self._apply_plot_limits()

    def update_telemetry(self, telemetry: Telemetry, started_at: float) -> None:
        self._telemetry = telemetry
        self.started_at = started_at

    def is_active(self) -> bool:
        return not self._closed and self.window.isVisible()

    def _on_window_changed(self, text: str) -> None:
        self._display_window = max(256, min(int(text), self.window_samples))

    def _on_fft_changed(self, text: str) -> None:
        self._current_fft_size = max(256, min(int(text), self.window_samples))
        self._analyzer.set_fft_size(self._current_fft_size)
        self._configure_spectrogram_buffers()
        self._analyzer.reset_detector()
        self._apply_plot_limits()

    def _configure_spectrogram_buffers(self) -> None:
        rows = max(1, self._current_fft_size)
        self._spectrogram_row_stride = max(1, (self._current_fft_size + rows - 1) // rows)
        self._spectrogram_rows = (self._current_fft_size + self._spectrogram_row_stride - 1) // self._spectrogram_row_stride
        self.spectrogram = self._np.full(
            (self._spectrogram_rows, self._spectrogram_columns),
            SPECTROGRAM_FLOOR_DB,
            dtype=self._np.float32,
        )
        self._spectrogram_storage = self._np.full(
            (self._spectrogram_rows, self._spectrogram_columns * 2),
            SPECTROGRAM_FLOOR_DB,
            dtype=self._np.float32,
        )
        self._spectrogram_view = self._spectrogram_storage[:, : self._spectrogram_columns]
        self._spectrogram_write_column = 0
        self._spectrogram_initialized = False
        self._spectrogram_rect_dirty = True

    def _on_doppler_span_changed(self, *_args) -> None:
        value = self.doppler_span_combo.currentData()
        self._analyzer.doppler_span_hz = None if value == "full" else float(value)
        self._apply_default_doppler_ranges()

    def _on_direction_changed(self, *_args) -> None:
        self._analyzer.direction_sign = float(self.direction_combo.currentData())

    def _on_notch_changed(self, *_args) -> None:
        self._analyzer.notch_width_hz = float(self.notch_combo.currentData())

    def _on_y_scale_changed(self, *_args) -> None:
        self._spectrum_scale_mode = str(self.y_scale_combo.currentData())
        if self._spectrum_scale_mode == "dbm":
            self._fft_display_top_db = FFT_DBM_DISPLAY_MAX_DBM
        else:
            self._fft_display_top_db = FFT_DB_DISPLAY_TOP_DB
        self._active_spectrum_lower = SPECTROGRAM_FLOOR_DB
        self._active_spectrum_upper = self._fft_display_top_db
        self._update_spectrum_axis_labels()
        self._apply_plot_limits()
        self._apply_default_y_ranges()

    def _update_spectrum_axis_labels(self) -> None:
        if self._spectrum_scale_mode == "dbm":
            self.fft_plot.setLabel("left", "Estimated level", units="dBm")
            self.cfar_plot.setLabel("left", "Residual / floor / lock", units="dBm")
        else:
            self.fft_plot.setLabel("left", "Excess over background", units="dB")
            self.cfar_plot.setLabel("left", "Residual / floor / lock", units="dB")

    def _on_remove_dc_toggled(self, checked: bool) -> None:
        self._analyzer.remove_dc = checked

    def _on_multi_target_toggled(self, checked: bool) -> None:
        self._analyzer.multi_target_enabled = checked

    def _on_max_targets_changed(self, text: str) -> None:
        self._analyzer.max_targets = int(text)

    def _on_acquire_snr_changed(self, value: float) -> None:
        self._analyzer.detector_acquire_snr_db = value

    def _on_release_snr_changed(self, value: float) -> None:
        self._analyzer.detector_release_snr_db = value

    def _on_noise_guard_mode_changed(self, *_args) -> None:
        self._analyzer.noise_guard_mode = str(self.noise_guard_combo.currentData())

    def _on_noise_avg_margin_changed(self, value: float) -> None:
        self._analyzer.noise_guard_avg_margin_db = value

    def _on_noise_max_margin_changed(self, value: float) -> None:
        self._analyzer.noise_guard_max_margin_db = value

    def _sync_autotuned_controls(self) -> None:
        generation = self._analyzer.autotune_generation
        if generation == self._last_autotune_generation:
            return
        self._last_autotune_generation = generation

        widgets = [
            self.acquire_snr_spin,
            self.release_snr_spin,
            self.noise_guard_combo,
            self.noise_avg_margin_spin,
            self.noise_max_margin_spin,
        ]
        for widget in widgets:
            widget.blockSignals(True)
        try:
            self.acquire_snr_spin.setValue(self._analyzer.detector_acquire_snr_db)
            self.release_snr_spin.setValue(self._analyzer.detector_release_snr_db)
            index = self.noise_guard_combo.findData(self._analyzer.noise_guard_mode)
            if index >= 0:
                self.noise_guard_combo.setCurrentIndex(index)
            self.noise_avg_margin_spin.setValue(self._analyzer.noise_guard_avg_margin_db)
            self.noise_max_margin_spin.setValue(self._analyzer.noise_guard_max_margin_db)
        finally:
            for widget in widgets:
                widget.blockSignals(False)

    def _spectrogram_levels(self):
        return (SPECTROGRAM_FLOOR_DB, WATERFALL_DISPLAY_TOP_DB)

    def _waterfall_display_line(self, fft_line):
        np = self._np
        line = np.asarray(fft_line, dtype=np.float32)
        line = np.maximum(line, SPECTROGRAM_FLOOR_DB)
        tau = max(1e-3, float(WATERFALL_COMPRESSION_TAU_DB))
        top = float(WATERFALL_DISPLAY_TOP_DB)
        compressed = top * (1.0 - np.exp(-line / tau))
        return np.clip(compressed, SPECTROGRAM_FLOOR_DB, top).astype(np.float32, copy=False)

    def _reduce_spectrogram_line(self, fft_line):
        line = self._np.asarray(fft_line, dtype=self._np.float32)
        if self._spectrogram_row_stride <= 1:
            return line
        expected = self._spectrogram_rows * self._spectrogram_row_stride
        if line.size < expected:
            line = self._np.pad(line, (0, expected - line.size), constant_values=SPECTROGRAM_FLOOR_DB)
        return line.reshape(self._spectrogram_rows, self._spectrogram_row_stride).max(axis=1).astype(self._np.float32, copy=False)

    def _apply_plot_limits(self) -> None:
        nyquist_hz = max(100.0, float(self.sample_rate_hz) * 0.5)
        self.spec_plot.setLimits(
            xMin=-float(self._spectrogram_columns),
            xMax=0.0,
            yMin=-nyquist_hz,
            yMax=nyquist_hz,
            minXRange=8.0,
            maxXRange=float(self._spectrogram_columns),
            minYRange=50.0,
            maxYRange=2.0 * nyquist_hz,
        )
        self.fft_plot.setLimits(
            xMin=-nyquist_hz,
            xMax=nyquist_hz,
            yMin=FFT_DBM_DISPLAY_MIN_DBM if self._spectrum_scale_mode == "dbm" else SPECTROGRAM_FLOOR_DB,
            yMax=FFT_DBM_DISPLAY_MAX_DBM if self._spectrum_scale_mode == "dbm" else FFT_Y_AXIS_LIMIT_TOP_DB,
            minXRange=50.0,
            maxXRange=2.0 * nyquist_hz,
            minYRange=2.0,
            maxYRange=(FFT_DBM_DISPLAY_MAX_DBM - FFT_DBM_DISPLAY_MIN_DBM) if self._spectrum_scale_mode == "dbm" else FFT_Y_AXIS_LIMIT_TOP_DB,
        )
        self._limits_sample_rate_hz = self.sample_rate_hz
        self.cfar_plot.setLimits(
            xMin=-nyquist_hz,
            xMax=nyquist_hz,
            yMin=FFT_DBM_DISPLAY_MIN_DBM if self._spectrum_scale_mode == "dbm" else SPECTROGRAM_FLOOR_DB,
            yMax=FFT_DBM_DISPLAY_MAX_DBM if self._spectrum_scale_mode == "dbm" else FFT_Y_AXIS_LIMIT_TOP_DB,
            minXRange=50.0,
            maxXRange=2.0 * nyquist_hz,
            minYRange=2.0,
            maxYRange=(FFT_DBM_DISPLAY_MAX_DBM - FFT_DBM_DISPLAY_MIN_DBM) if self._spectrum_scale_mode == "dbm" else FFT_Y_AXIS_LIMIT_TOP_DB,
        )
        self.filter_plot.setLimits(
            xMin=-nyquist_hz,
            xMax=nyquist_hz,
            yMin=0.0,
            yMax=36.0,
            minXRange=50.0,
            maxXRange=2.0 * nyquist_hz,
            minYRange=2.0,
            maxYRange=36.0,
        )
        self._apply_default_doppler_ranges()
        self._spectrogram_rect_dirty = True

    def _default_doppler_range(self) -> Tuple[float, float]:
        nyquist_hz = max(100.0, float(self.sample_rate_hz) * 0.5)
        span = self._analyzer.doppler_span_hz
        if span is None:
            return -nyquist_hz, nyquist_hz
        clipped = min(float(span), nyquist_hz)
        return -clipped, clipped

    def _apply_default_doppler_ranges(self) -> None:
        minimum_hz, maximum_hz = self._default_doppler_range()
        self._fft_viewbox.setXRange(minimum_hz, maximum_hz, padding=0.0)
        self._cfar_viewbox.setXRange(minimum_hz, maximum_hz, padding=0.0)
        self._spec_viewbox.setYRange(minimum_hz, maximum_hz, padding=0.0)
        self.filter_plot.setXRange(minimum_hz, maximum_hz, padding=0.0)

    def _apply_default_y_ranges(self) -> None:
        if self._spectrum_scale_mode == "dbm":
            lower = FFT_DBM_DISPLAY_MIN_DBM
            upper = self._active_spectrum_upper
        else:
            lower = self._active_spectrum_lower
            upper = self._active_spectrum_upper
        self._fft_viewbox.setYRange(lower, upper, padding=0.0)
        self._cfar_viewbox.setYRange(lower, upper, padding=0.0)

    def _spectrum_plot_arrays(self, snapshot: AnalysisSnapshot):
        if self._spectrum_scale_mode == "dbm":
            return (
                snapshot.fft_dbm,
                snapshot.cfar_noise_floor_dbm,
                snapshot.cfar_threshold_dbm,
                [target.absolute_dbm for target in snapshot.targets],
            )
        return (
            snapshot.fft_magnitude,
            snapshot.cfar_noise_floor,
            snapshot.cfar_threshold,
            [target.display_db for target in snapshot.targets],
        )

    def _update_spectrum_headroom(self, fft_curve_values, cfar_noise_values, cfar_threshold_values, target_y_values) -> None:
        np = self._np
        parts = [fft_curve_values]
        if cfar_noise_values is not None:
            parts.append(cfar_noise_values)
        if cfar_threshold_values is not None:
            parts.append(cfar_threshold_values)
        if target_y_values:
            parts.append(np.asarray(target_y_values, dtype=np.float32))

        finite_maxima = []
        finite_noise_reference = []
        for part in parts:
            if part is None:
                continue
            values = np.asarray(part, dtype=np.float32)
            finite = values[np.isfinite(values)]
            if finite.size:
                finite_maxima.append(float(np.max(finite)))
        if cfar_noise_values is not None:
            values = np.asarray(cfar_noise_values, dtype=np.float32)
            finite = values[np.isfinite(values)]
            if finite.size:
                finite_noise_reference.append(float(np.median(finite)))
        if fft_curve_values is not None:
            values = np.asarray(fft_curve_values, dtype=np.float32)
            finite = values[np.isfinite(values)]
            if finite.size:
                finite_noise_reference.append(float(np.percentile(finite, 18.0)))

        if self._spectrum_scale_mode == "dbm":
            lower = FFT_DBM_DISPLAY_MIN_DBM
            default_upper = FFT_DBM_DISPLAY_MAX_DBM
            self._active_spectrum_lower = lower
            self._active_spectrum_upper = default_upper
            self._fft_viewbox.setYRange(lower, default_upper, padding=0.0)
            self._cfar_viewbox.setYRange(lower, default_upper, padding=0.0)
            return
        else:
            hard_lower = SPECTROGRAM_FLOOR_DB
            default_upper = self._fft_display_top_db
            padding = 2.0
            target_fraction = 0.32
            decay_alpha = 0.02
            max_drop_db_per_update = 0.35
            lower_rise_alpha = 0.06
            lower_fall_alpha = 0.02
            max_raise_db_per_update = 0.30
            max_fall_db_per_update = 0.12
            background_margin_db = 3.5

        observed_peak = max(finite_maxima) if finite_maxima else hard_lower
        desired_upper = max(default_upper, (observed_peak - hard_lower) / target_fraction + hard_lower + padding)
        desired_upper = min(max(default_upper, desired_upper), 150.0)

        if desired_upper >= self._active_spectrum_upper:
            self._active_spectrum_upper = desired_upper
        else:
            relaxed_upper = (1.0 - decay_alpha) * self._active_spectrum_upper + decay_alpha * desired_upper
            limited_upper = max(self._active_spectrum_upper - max_drop_db_per_update, relaxed_upper)
            self._active_spectrum_upper = max(default_upper, limited_upper)

        noise_reference = max(finite_noise_reference) if finite_noise_reference else hard_lower
        desired_lower = max(hard_lower, noise_reference - background_margin_db)
        desired_lower = min(desired_lower, self._active_spectrum_upper - 8.0)
        if desired_lower >= self._active_spectrum_lower:
            relaxed_lower = (1.0 - lower_rise_alpha) * self._active_spectrum_lower + lower_rise_alpha * desired_lower
            limited_lower = min(self._active_spectrum_lower + max_raise_db_per_update, relaxed_lower)
        else:
            relaxed_lower = (1.0 - lower_fall_alpha) * self._active_spectrum_lower + lower_fall_alpha * desired_lower
            limited_lower = max(self._active_spectrum_lower - max_fall_db_per_update, relaxed_lower)
        self._active_spectrum_lower = max(hard_lower, min(limited_lower, self._active_spectrum_upper - 8.0))

        self._fft_viewbox.setYRange(self._active_spectrum_lower, self._active_spectrum_upper, padding=0.0)
        self._cfar_viewbox.setYRange(self._active_spectrum_lower, self._active_spectrum_upper, padding=0.0)

    def _reset_view_ranges(self) -> None:
        self._apply_default_doppler_ranges()
        self._apply_default_y_ranges()
        self.filter_plot.setYRange(0.0, 24.0, padding=0.0)

    def _render_target_log(self) -> None:
        now = time.monotonic()
        if (now - self._last_target_log_render) < 0.5:
            return
        visible_done_events = [event for event in self._target_events if event["duration_s"] >= 1.0]
        visible_live_events = [event for event in self._active_target_events if event["duration_s"] >= 1.0]
        if not visible_done_events and not visible_live_events:
            text = "No target history yet."
            if text == self._last_target_log_text:
                return
            self.target_log.setPlainText(text)
            self._last_target_log_text = text
            self._last_target_log_render = now
            return

        def state_code(event: dict) -> str:
            return "L" if event["status"] == "live" else "D"

        lines = [
            " ID  ST    t0      dur[s]   f_start    f_stop    f_mean      v[km/h]   dir",
            "---- --  --------  -------  --------  --------  --------  ----------  ----",
        ]
        for event in visible_done_events[-12:]:
            lines.append(
                f"{event['id']:4d}  {state_code(event):<2}  {event['timestamp']:<8}  {event['duration_s']:7.1f}  "
                f"{event['start_hz']:8.1f}  {event['stop_hz']:8.1f}  {event['doppler_hz']:8.1f}  "
                f"{event['velocity_kmh']:10.2f}  {event['direction_code']:<4}"
            )
        for event in sorted(visible_live_events, key=lambda item: item["started_at"]):
            lines.append(
                f"{event['id']:4d}  {state_code(event):<2}  {event['timestamp']:<8}  {event['duration_s']:7.1f}  "
                f"{event['start_hz']:8.1f}  {event['stop_hz']:8.1f}  {event['doppler_hz']:8.1f}  "
                f"{event['velocity_kmh']:10.2f}  {event['direction_code']:<4}"
            )
        text = "\n".join(lines)
        if text == self._last_target_log_text:
            return
        self.target_log.setPlainText(text)
        self._last_target_log_text = text
        self._last_target_log_render = now
        scrollbar = self.target_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _curve_view(self, freq_axis, values):
        if freq_axis is None or values is None:
            return None, None
        return freq_axis, values

    def _update_spectrogram_geometry(self, snapshot: AnalysisSnapshot) -> None:
        if not self._spectrogram_rect_dirty or snapshot.freq_axis is None:
            return
        self.spec_image.setRect(
            self._QtCore.QRectF(
                -self._spectrogram_columns,
                float(snapshot.freq_axis[0]),
                self._spectrogram_columns,
                float(snapshot.freq_axis[-1] - snapshot.freq_axis[0]),
            )
        )
        self.spec_plot.setXRange(-self._spectrogram_columns, 0.0, padding=0.0)
        self._spectrogram_rect_dirty = False

    def _target_match_window_hz(self, doppler_hz: float) -> float:
        return max(220.0, 0.22 * abs(doppler_hz) + 55.0)

    def _target_velocity_window_kmh(self, velocity_kmh: float) -> float:
        return max(3.0, 0.25 * abs(velocity_kmh) + 1.2)

    def _direction_code(self, direction_label: str) -> str:
        mapping = {
            "approaching": "APP",
            "receding": "REC",
            "stationary": "STA",
            "unknown": "UNK",
        }
        return mapping.get(direction_label, direction_label[:4].upper())

    def _target_matches_event(self, event: dict, target) -> bool:
        doppler_delta = min(
            abs(target.doppler_hz - event["doppler_hz"]),
            abs(target.doppler_hz - event["stop_hz"]),
        )
        if doppler_delta > self._target_match_window_hz(target.doppler_hz):
            return False
        if abs(target.velocity_kmh - event["velocity_kmh"]) > self._target_velocity_window_kmh(target.velocity_kmh):
            return False
        if event["direction"] != target.direction_label:
            direction_pair = {event["direction"], target.direction_label}
            if "stationary" not in direction_pair:
                return False
        return True

    def _target_match_score(self, event: dict, target) -> float:
        doppler_delta = min(
            abs(target.doppler_hz - event["doppler_hz"]),
            abs(target.doppler_hz - event["stop_hz"]),
        )
        velocity_delta = abs(target.velocity_kmh - event["velocity_kmh"])
        doppler_score = doppler_delta / self._target_match_window_hz(target.doppler_hz)
        velocity_score = velocity_delta / self._target_velocity_window_kmh(target.velocity_kmh)
        return doppler_score + 0.7 * velocity_score

    def _new_target_event(self, target, now: float) -> dict:
        event = {
            "id": self._next_target_event_id,
            "started_at": now,
            "last_seen": now,
            "timestamp": time.strftime("%H:%M:%S", time.localtime()),
            "doppler_hz": target.doppler_hz,
            "start_hz": target.doppler_hz,
            "stop_hz": target.doppler_hz,
            "velocity_kmh": target.velocity_kmh,
            "direction": target.direction_label,
            "direction_code": self._direction_code(target.direction_label),
            "duration_s": 0.0,
            "status": "live",
        }
        self._next_target_event_id += 1
        return event

    def _update_target_event(self, event: dict, target, now: float) -> None:
        alpha = 0.18
        event["last_seen"] = now
        event["doppler_hz"] = (1.0 - alpha) * event["doppler_hz"] + alpha * target.doppler_hz
        event["stop_hz"] = target.doppler_hz
        event["velocity_kmh"] = (1.0 - alpha) * event["velocity_kmh"] + alpha * target.velocity_kmh
        event["direction"] = target.direction_label
        event["direction_code"] = self._direction_code(target.direction_label)
        event["duration_s"] = now - event["started_at"]
        event["status"] = "live"

    def _reopen_recent_target_event(self, target, now: float) -> Optional[dict]:
        max_gap_s = 3.5
        for index in range(len(self._target_events) - 1, -1, -1):
            event = self._target_events[index]
            if (now - event["last_seen"]) > max_gap_s:
                break
            if self._target_matches_event(event, target):
                self._target_events.pop(index)
                event["status"] = "live"
                return event
        return None

    def _close_stale_target_events(self, now: float, force_all: bool = False, timeout_s: float = 0.8) -> None:
        closed_any = False
        keep_events = []
        for event in self._active_target_events:
            if force_all or (now - event["last_seen"]) > timeout_s:
                if event["duration_s"] >= 1.0:
                    event["status"] = "done"
                    self._target_events.append(event)
                    closed_any = True
            else:
                keep_events.append(event)
        self._active_target_events = keep_events
        if closed_any:
            self._render_target_log()

    def _update_target_history(self, snapshot: AnalysisSnapshot, now: float) -> None:
        if snapshot.detector_state in {"warming", "calibrating"}:
            return
        if snapshot.detector_state == "coasting":
            self._close_stale_target_events(now, timeout_s=1.8)
            return
        current_targets = list(snapshot.targets) if (snapshot.detector_state == "locked" and snapshot.targets) else []
        if not current_targets:
            self._close_stale_target_events(now)
            return

        unmatched_active = list(self._active_target_events)
        updated_active: List[dict] = []
        matched_target_ids = set()
        for event in self._active_target_events:
            best_index = None
            best_score = None
            for index, target in enumerate(current_targets):
                if index in matched_target_ids or not self._target_matches_event(event, target):
                    continue
                score = self._target_match_score(event, target)
                if best_score is None or score < best_score:
                    best_score = score
                    best_index = index
            if best_index is None:
                continue
            matched_target_ids.add(best_index)
            target = current_targets[best_index]
            self._update_target_event(event, target, now)
            updated_active.append(event)
            if event in unmatched_active:
                unmatched_active.remove(event)

        for index, target in enumerate(current_targets):
            if index in matched_target_ids:
                continue
            event = self._reopen_recent_target_event(target, now)
            if event is None:
                event = self._new_target_event(target, now)
            self._update_target_event(event, target, now)
            updated_active.append(event)

        self._active_target_events = updated_active
        for event in unmatched_active:
            if (now - event["last_seen"]) > 0.8:
                if event["duration_s"] >= 1.0:
                    event["status"] = "done"
                    self._target_events.append(event)
            else:
                self._active_target_events.append(event)
        self._render_target_log()

    def _update_cfar_annotation(
        self,
        snapshot: AnalysisSnapshot,
        fft_curve_values,
        cfar_noise_values,
        cfar_threshold_values,
    ) -> None:
        if snapshot.freq_axis is None or fft_curve_values is None or cfar_threshold_values is None:
            self.cfar_info.setText("Tracker idle")
            self.cfar_primary_target.setData([], [])
            return

        info_lines = []
        if snapshot.targets:
            primary = snapshot.targets[0]
            if self._spectrum_scale_mode == "dbm":
                signal_level = float(primary.absolute_dbm)
            else:
                signal_level = float(primary.display_db)
            floor_level = None
            if cfar_noise_values is not None and primary.bin_index < len(cfar_noise_values):
                floor_candidate = cfar_noise_values[primary.bin_index]
                if self._np.isfinite(floor_candidate):
                    floor_level = float(floor_candidate)
            threshold_level = None
            if primary.bin_index < len(cfar_threshold_values):
                threshold_candidate = cfar_threshold_values[primary.bin_index]
                if self._np.isfinite(threshold_candidate):
                    threshold_level = float(threshold_candidate)
            margin = (signal_level - threshold_level) if threshold_level is not None else None
            unit = "dBm" if self._spectrum_scale_mode == "dbm" else "dB"
            info_lines.append(f"Track {primary.doppler_hz:+.1f} Hz")
            info_lines.append(f"Signal {signal_level:.1f} {unit}")
            if floor_level is not None:
                info_lines.append(f"Floor {floor_level:.1f} {unit}")
            if threshold_level is not None and margin is not None:
                info_lines.append(f"Lock {threshold_level:.1f} {unit}")
                info_lines.append(f"Margin {margin:+.1f} {unit}")
            else:
                info_lines.append("Lock n/a")
            self.cfar_primary_target.setData([primary.doppler_hz], [signal_level])
        else:
            info_lines.append("No validated track")
            info_lines.append("Waiting for residual inside lock gate")
            self.cfar_primary_target.setData([], [])
        if snapshot.tracker_predicted_hz != 0.0 or snapshot.detector_state == "locked":
            info_lines.append(f"Pred {snapshot.tracker_predicted_hz:+.1f} Hz")
            info_lines.append(f"Gate +/-{snapshot.tracker_gate_hz:.1f} Hz")
            info_lines.append(f"Innov {snapshot.tracker_innovation_hz:+.1f} Hz")
        info_lines.append(f"State {snapshot.tracker_phase}")
        info_lines.append(f"Score {snapshot.tracker_score:.1f}")
        self.cfar_info.setText("\n".join(info_lines))
        x_min, x_max = self._cfar_viewbox.viewRange()[0]
        y_min, y_max = self._cfar_viewbox.viewRange()[1]
        self.cfar_info.setPos(x_min + 0.02 * (x_max - x_min), y_max - 0.05 * (y_max - y_min))

    def _update_filter_annotation(self, snapshot: AnalysisSnapshot) -> None:
        if snapshot.freq_axis is None or snapshot.filter_attenuation_db is None:
            self.filter_info.setText("Filter idle")
            return

        np = self._np
        attenuation = np.asarray(snapshot.filter_attenuation_db, dtype=np.float32)
        spur_profile = (
            np.asarray(snapshot.spur_profile_db, dtype=np.float32)
            if snapshot.spur_profile_db is not None
            else np.zeros_like(attenuation)
        )
        finite_attenuation = attenuation[np.isfinite(attenuation)]
        finite_spur = spur_profile[np.isfinite(spur_profile)]
        if finite_attenuation.size == 0:
            self.filter_info.setText("Filter idle")
            return

        active_bins = int(np.count_nonzero(finite_attenuation >= 1.0))
        max_attenuation = float(np.max(finite_attenuation))
        mean_active = float(np.mean(finite_attenuation[finite_attenuation >= 1.0])) if active_bins else 0.0
        max_spur = float(np.max(finite_spur)) if finite_spur.size else 0.0
        info_lines = [
            f"Profiled lines {active_bins}",
            f"Peak attenuation {max_attenuation:.1f} dB",
            f"Active mean {mean_active:.1f} dB" if active_bins else "Active mean n/a",
            f"Stable spur evidence {max_spur:.1f} dB",
        ]
        self.filter_info.setText("\n".join(info_lines))
        x_min, x_max = self.filter_plot.getViewBox().viewRange()[0]
        y_min, y_max = self.filter_plot.getViewBox().viewRange()[1]
        self.filter_info.setPos(x_min + 0.02 * (x_max - x_min), y_max - 0.05 * (y_max - y_min))

    def _toggle_pause(self) -> None:
        self._paused = not self._paused
        self.pause_button.setText("Resume" if self._paused else "Pause")
        self.mode_badge.setText("PAUSED" if self._paused else "LIVE")

    def _reset_buffers(self) -> None:
        self.i_buffer.clear()
        self.q_buffer.clear()
        self._configure_spectrogram_buffers()
        self.spec_image.setImage(self._spectrogram_view, autoLevels=False, levels=self._spectrogram_levels())
        self._analyzer.reset_detector()
        self._last_snapshot = None

    def _clear_target_log(self) -> None:
        self._target_events.clear()
        self._active_target_events.clear()
        self._last_target_log_render = 0.0
        self._render_target_log()

    def _push_spectrogram_column(self, fft_line) -> None:
        fft_line = self._reduce_spectrogram_line(fft_line)
        if not self._spectrogram_initialized:
            self.spectrogram[:, :] = fft_line[:, None]
            self._spectrogram_storage[:, : self._spectrogram_columns] = fft_line[:, None]
            self._spectrogram_storage[:, self._spectrogram_columns :] = fft_line[:, None]
            self._spectrogram_view = self._spectrogram_storage[:, : self._spectrogram_columns]
            self._spectrogram_write_column = 0
            self._spectrogram_initialized = True
            return

        self.spectrogram[:, self._spectrogram_write_column] = fft_line
        write_column = self._spectrogram_write_column
        self._spectrogram_storage[:, write_column] = fft_line
        self._spectrogram_storage[:, write_column + self._spectrogram_columns] = fft_line
        self._spectrogram_write_column = (write_column + 1) % self._spectrogram_columns
        self._spectrogram_view = self._spectrogram_storage[
            :, self._spectrogram_write_column : self._spectrogram_write_column + self._spectrogram_columns
        ]

    def _save_snapshot(self) -> None:
        snapshot_dir = Path("artifacts/ui_snapshots")
        snapshot_dir.mkdir(parents=True, exist_ok=True)
        path = snapshot_dir / f"radar_ui_{timestamp_for_filename()}.png"
        pixmap = self.window.grab()
        pixmap.save(str(path))
        self.status_bar.showMessage(f"Snapshot saved to {path}", 5000)

    def _export_metrics(self) -> None:
        if self._last_snapshot is None:
            self.status_bar.showMessage("No analyzed frame available yet.", 4000)
            return
        export_dir = Path("artifacts/exports")
        export_dir.mkdir(parents=True, exist_ok=True)
        path = export_dir / f"radar_metrics_{timestamp_for_filename()}.json"
        payload = {
            "created_at_unix": time.time(),
            "carrier_hz": RADAR_CARRIER_HZ,
            "sample_rate_hz": self.sample_rate_hz,
            "source": self.source_name,
            "detector": self._last_snapshot.detector_state,
            "dominant_hz": self._last_snapshot.dominant_hz,
            "velocity_mps": self._last_snapshot.velocity_mps,
            "velocity_kmh": self._last_snapshot.velocity_kmh,
            "direction": self._last_snapshot.direction_label,
            "peak_snr_db": self._last_snapshot.peak_snr_db,
            "targets": [
                {
                    "doppler_hz": target.doppler_hz,
                    "snr_db": target.snr_db,
                    "velocity_mps": target.velocity_mps,
                    "velocity_kmh": target.velocity_kmh,
                    "direction": target.direction_label,
                }
                for target in self._last_snapshot.targets
            ],
            "rms_iq": self._last_snapshot.rms_value,
            "phase_std_rad": self._last_snapshot.phase_std,
            "motion_index": self._last_snapshot.motion_index,
            "spectral_spread_hz": self._last_snapshot.spectral_spread,
            "coherence": self._last_snapshot.coherence,
            "telemetry": {
                "crc_errors": self._telemetry.crc_errors,
                "uncorrectable_errors": self._telemetry.uncorrectable_errors,
                "sequence_gaps": self._telemetry.sequence_gaps,
                "device_tx_backpressure": self._telemetry.device_tx_backpressure,
                "device_adc_event_overflows": self._telemetry.device_adc_event_overflows,
            },
        }
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        self.status_bar.showMessage(f"Metrics exported to {path}", 5000)

    def _update_cards(
        self,
        fps: float,
        dominant_hz: float,
        rms_value: float,
        phase_std: float,
        motion_index: float,
        spectral_spread: float,
        coherence: float,
        scene_label: str,
        scene_confidence: float,
        velocity_mps: float,
        velocity_kmh: float,
        direction_label: str,
        peak_snr_db: float,
        detector_state: str,
    ) -> None:
        elapsed = time.monotonic() - self.started_at
        values = {
            "fps": f"{fps:0.1f}",
            "sample_rate": f"{self.sample_rate_hz/1000.0:0.1f} kHz",
            "scene": f"{scene_label} ({scene_confidence*100.0:0.0f}%)",
            "dominant_hz": f"{dominant_hz:0.1f} Hz" if detector_state == "locked" else "-",
            "velocity": f"{velocity_mps:+0.2f} m/s  {velocity_kmh:+0.1f} km/h" if detector_state == "locked" else "-",
            "direction": direction_label if detector_state == "locked" else "-",
            "snr": f"{peak_snr_db:0.1f} dB" if detector_state == "locked" else "-",
            "detector": detector_state,
            "targets": str(len(self._last_snapshot.targets)) if self._last_snapshot is not None else "0",
            "rms": f"{rms_value:0.1f}",
            "phase_std": f"{phase_std:0.3f} rad",
            "motion_index": f"{motion_index:0.3f}",
            "spectral_spread": f"{spectral_spread:0.1f} Hz",
            "coherence": f"{coherence:0.3f}",
            "device_drops": f"{self._telemetry.device_tx_backpressure}/{self._telemetry.device_adc_event_overflows}",
            "crc": f"{self._telemetry.crc_errors}/{self._telemetry.uncorrectable_errors}",
            "recording": (
                self._telemetry.current_recording_name
                if self.recording_enabled and self._telemetry.current_recording_name != "-"
                else "armed"
                if self.recording_enabled
                else "disabled"
            ),
        }
        for key, text in values.items():
            if self._last_card_texts.get(key) == text:
                continue
            label = self._card_values[key]
            label.setText(text)
            label.setToolTip(text)
            self._last_card_texts[key] = text
        if self._telemetry.uncorrectable_errors > 0 or self._telemetry.device_stream_fault != 0:
            quality_text = "QUALITY DEGRADED"
        elif self._telemetry.crc_errors > 0 or self._telemetry.device_tx_backpressure > 0:
            quality_text = "QUALITY CAUTION"
        else:
            quality_text = "QUALITY OK"
        if quality_text != self._last_quality_badge:
            self.quality_badge.setText(quality_text)
            self._last_quality_badge = quality_text
        if detector_state == "warming":
            lock_text = "WARMING"
        elif detector_state == "calibrating":
            lock_text = "CALIBRATING"
        elif detector_state == "locked":
            lock_text = "LOCKED"
        elif detector_state == "coasting":
            lock_text = "COASTING"
        elif detector_state == "tentative":
            lock_text = "TENTATIVE"
        else:
            lock_text = "SEARCHING"
        if lock_text != self._last_lock_badge:
            self.lock_badge.setText(lock_text)
            self._last_lock_badge = lock_text
        status_text = "  ".join(
            [
                f"Duration {format_duration(elapsed)}",
                f"Source {self.source_name}",
                f"Frames {self._telemetry.frames_ok + self._telemetry.frames_corrected}",
                f"CRC {self._telemetry.crc_errors}",
                f"SeqGap {self._telemetry.sequence_gaps}",
                f"DevDrop {self._telemetry.device_tx_backpressure}/{self._telemetry.device_adc_event_overflows}",
            ]
        )
        if status_text != self._last_status_message:
            self.status_bar.showMessage(status_text)
            self._last_status_message = status_text

    def draw(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self.last_draw) < self.refresh_period:
            return
        if len(self.i_buffer) < 16:
            return
        if self._paused:
            return

        analysis_due = force or self._last_snapshot is None or (now - self._last_analysis_at) >= self.analysis_period
        if analysis_due:
            snapshot = self._analyzer.analyze(self.i_buffer, self.q_buffer, self.sample_rate_hz, self._display_window)
            if snapshot is None:
                return
            self._last_snapshot = snapshot
            self._last_analysis_at = now
        else:
            snapshot = self._last_snapshot
            if snapshot is None:
                return

        if analysis_due and snapshot.freq_axis is not None and snapshot.fft_magnitude is not None:
            fft_curve_values, cfar_noise_values, cfar_threshold_values, target_y_values = self._spectrum_plot_arrays(snapshot)
            self._update_spectrum_headroom(fft_curve_values, cfar_noise_values, cfar_threshold_values, target_y_values)
            fft_x, fft_y = self._curve_view(snapshot.freq_axis, fft_curve_values)
            self.fft_curve.setData(fft_x, fft_y)
            if snapshot.detector_state == "locked":
                self.fft_peak_line.setValue(snapshot.dominant_hz)
                self.spec_peak_line.setValue(snapshot.dominant_hz)
                self.cfar_peak_line.setValue(snapshot.dominant_hz)
                self.fft_peak_line.show()
                self.spec_peak_line.show()
                self.cfar_peak_line.show()
            else:
                self.fft_peak_line.hide()
                self.spec_peak_line.hide()
                self.cfar_peak_line.hide()
            if snapshot.tracker_gate_hz > 0.0 and (snapshot.tracker_predicted_hz != 0.0 or snapshot.detector_state == "locked"):
                center_hz = snapshot.tracker_predicted_hz if snapshot.tracker_predicted_hz != 0.0 else snapshot.dominant_hz
                self.cfar_gate_left.setValue(center_hz - snapshot.tracker_gate_hz)
                self.cfar_gate_right.setValue(center_hz + snapshot.tracker_gate_hz)
                self.cfar_gate_left.show()
                self.cfar_gate_right.show()
            else:
                self.cfar_gate_left.hide()
                self.cfar_gate_right.hide()

            secondary_targets = snapshot.targets[1:] if len(snapshot.targets) > 1 else []
            for index, line in enumerate(self._secondary_fft_lines):
                if index < len(secondary_targets):
                    line.setValue(secondary_targets[index].doppler_hz)
                    line.show()
                else:
                    line.hide()
            for index, line in enumerate(self._secondary_spec_lines):
                if index < len(secondary_targets):
                    line.setValue(secondary_targets[index].doppler_hz)
                    line.show()
                else:
                    line.hide()

            fft_line = self._waterfall_display_line(snapshot.fft_magnitude)
            self._push_spectrogram_column(fft_line)
            self.spec_image.setImage(self._spectrogram_view, autoLevels=False, levels=self._spectrogram_levels())
            self._update_spectrogram_geometry(snapshot)
            cfar_x, cfar_signal_y = self._curve_view(snapshot.freq_axis, fft_curve_values)
            self.cfar_signal_curve.setData(cfar_x, cfar_signal_y)
            if cfar_noise_values is not None:
                cfar_noise_x, cfar_noise_y = self._curve_view(snapshot.freq_axis, cfar_noise_values)
                self.cfar_noise_curve.setData(cfar_noise_x, cfar_noise_y)
            else:
                self.cfar_noise_curve.setData([], [])
            if cfar_threshold_values is not None:
                cfar_threshold_x, cfar_threshold_y = self._curve_view(snapshot.freq_axis, cfar_threshold_values)
                self.cfar_threshold_curve.setData(cfar_threshold_x, cfar_threshold_y)
            else:
                self.cfar_threshold_curve.setData([], [])
            if snapshot.targets:
                self.cfar_targets.setData(
                    x=[target.doppler_hz for target in snapshot.targets],
                    y=target_y_values,
                )
            else:
                self.cfar_targets.setData([], [])
            self._update_cfar_annotation(snapshot, fft_curve_values, cfar_noise_values, cfar_threshold_values)
            if snapshot.filter_attenuation_db is not None:
                filter_x, filter_y = self._curve_view(snapshot.freq_axis, snapshot.filter_attenuation_db)
                self.filter_attenuation_curve.setData(filter_x, filter_y)
            else:
                self.filter_attenuation_curve.setData([], [])
            if snapshot.spur_profile_db is not None:
                spur_x, spur_y = self._curve_view(snapshot.freq_axis, snapshot.spur_profile_db)
                self.filter_spur_curve.setData(spur_x, spur_y)
            else:
                self.filter_spur_curve.setData([], [])
            self._update_filter_annotation(snapshot)
        elif analysis_due:
            for line in self._secondary_fft_lines:
                line.hide()
            for line in self._secondary_spec_lines:
                line.hide()
            self.cfar_peak_line.hide()
            self.cfar_gate_left.hide()
            self.cfar_gate_right.hide()
            self.cfar_signal_curve.setData([], [])
            self.cfar_noise_curve.setData([], [])
            self.cfar_threshold_curve.setData([], [])
            self.cfar_targets.setData([], [])
            self.cfar_primary_target.setData([], [])
            self.cfar_info.setText("Tracker idle")
            self.filter_attenuation_curve.setData([], [])
            self.filter_spur_curve.setData([], [])
            self.filter_info.setText("Filter idle")

        if analysis_due and (force or (now - self._last_status_update) >= STATUS_REFRESH_S):
            fps = (self._telemetry.frames_ok + self._telemetry.frames_corrected) / max(1e-6, now - self.started_at)
            self._sync_autotuned_controls()
            self._update_target_history(snapshot, now)
            self._update_cards(
                fps=fps,
                dominant_hz=snapshot.dominant_hz,
                rms_value=snapshot.rms_value,
                phase_std=snapshot.phase_std,
                motion_index=snapshot.motion_index,
                spectral_spread=snapshot.spectral_spread,
                coherence=snapshot.coherence,
                scene_label=snapshot.scene_label,
                scene_confidence=snapshot.scene_confidence,
                velocity_mps=snapshot.velocity_mps,
                velocity_kmh=snapshot.velocity_kmh,
                direction_label=snapshot.direction_label,
                peak_snr_db=snapshot.peak_snr_db,
                detector_state=snapshot.detector_state,
            )
            self._last_status_update = now
        self.last_draw = now

    def process_events(self) -> None:
        self.app.processEvents()

    def close(self) -> None:
        if self.window.isVisible():
            self.window.close()
