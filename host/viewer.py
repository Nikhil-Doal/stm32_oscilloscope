import struct
import sys
import time
import threading
from dataclasses import dataclass

import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# --- Frame protocol constants (must match firmware) -------------------------
SYNC_0 = 0xAA
SYNC_1 = 0x55
FRAME_TYPE_WAVEFORM = 0x01

WAVEFORM_SAMPLES = 2048
MAG_BINS = 512

PAYLOAD_HDR_BYTES = 16
PAYLOAD_BYTES = PAYLOAD_HDR_BYTES + WAVEFORM_SAMPLES * 2 + MAG_BINS * 4
FRAME_OVERHEAD = 7
FRAME_TOTAL = FRAME_OVERHEAD + PAYLOAD_BYTES

PORT = "COM3"


# --- CRC16-CCITT (polynomial 0x1021, init 0xFFFF) ---------------------------
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# --- Parsed frame structure -------------------------------------------------
@dataclass
class Frame:
    tick_ms: int
    sample_rate_hz: int
    peak_bin: int
    peak_mag: float
    samples: np.ndarray
    magnitudes: np.ndarray


# --- Streaming parser: feed bytes, get frames ------------------------------
class FrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.frames_parsed = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        while True:
            frame = self._try_parse_one()
            if frame is None:
                return
            yield frame

    def _try_parse_one(self):
        while len(self.buf) >= 2:
            if self.buf[0] == SYNC_0 and self.buf[1] == SYNC_1:
                break
            del self.buf[0]

        if len(self.buf) < FRAME_TOTAL:
            return None

        payload_len = self.buf[3] | (self.buf[4] << 8)
        if payload_len != PAYLOAD_BYTES:
            del self.buf[0]
            return None

        crc_region = bytes(self.buf[2:2 + 3 + PAYLOAD_BYTES])
        crc_received = self.buf[FRAME_TOTAL - 2] | (self.buf[FRAME_TOTAL - 1] << 8)
        crc_expected = crc16_ccitt(crc_region)

        if crc_received != crc_expected:
            self.crc_errors += 1
            del self.buf[0]
            return None

        off = 5
        tick, fs, peak_bin = struct.unpack_from("<III", self.buf, off)
        peak_mag, = struct.unpack_from("<f", self.buf, off + 12)

        samples_off = off + PAYLOAD_HDR_BYTES
        samples = np.frombuffer(
            self.buf, dtype=np.uint16,
            count=WAVEFORM_SAMPLES,
            offset=samples_off,
        ).copy()

        mags_off = samples_off + WAVEFORM_SAMPLES * 2
        mags = np.frombuffer(
            self.buf, dtype=np.float32,
            count=MAG_BINS,
            offset=mags_off,
        ).copy()

        del self.buf[:FRAME_TOTAL]
        self.frames_parsed += 1

        return Frame(
            tick_ms=tick,
            sample_rate_hz=fs,
            peak_bin=peak_bin,
            peak_mag=peak_mag,
            samples=samples,
            magnitudes=mags,
        )


# --- Serial reader thread ---------------------------------------------------
class SerialReader(threading.Thread):
    def __init__(self, port: str, frame_callback):
        super().__init__(daemon=True)
        self.port = port
        self.frame_callback = frame_callback
        self.running = True
        self.parser = FrameParser()
        self.bytes_read = 0

    def run(self):
        try:
            ser = serial.Serial(self.port, timeout=0.05)
        except serial.SerialException as e:
            print(f"failed to open {self.port}: {e}", file=sys.stderr)
            return

        while self.running:
            try:
                chunk = ser.read(8192)
                if chunk:
                    self.bytes_read += len(chunk)
                    for frame in self.parser.feed(chunk):
                        self.frame_callback(frame)
            except serial.SerialException:
                print("serial disconnected", file=sys.stderr)
                break

        ser.close()

    def stop(self):
        self.running = False


# --- Trigger helper ---------------------------------------------------------
def find_rising_edge(samples: np.ndarray, level_uint16: int, min_amplitude: int = 200) -> int:
    """
    Find the first rising edge through level. Returns -1 if no usable edge
    (no crossing, or signal amplitude below min_amplitude — avoids triggering on noise).
    """
    # Reject if signal swing is too small — avoids "locking" on random noise.
    if int(samples.max()) - int(samples.min()) < min_amplitude:
        return -1

    level = np.uint16(level_uint16)
    prev = samples[:-1]
    curr = samples[1:]
    mask = (prev < level) & (curr >= level)
    if not mask.any():
        return -1
    return int(np.argmax(mask)) + 1

# --- GUI --------------------------------------------------------------------
class Viewer(QtWidgets.QMainWindow):
    def __init__(self, port: str):
        super().__init__()
        self.setWindowTitle("STM32H7 Oscilloscope")
        self.resize(1500, 800)

        self.latest_frame = None
        self.latest_frame_lock = threading.Lock()
        self.displayed_frames = 0
        self.last_fps_check = time.monotonic()
        self.current_fps = 0.0
        self.paused = False

        # Display state
        self.db_scale = False
        self.trigger_enabled = True
        self.trigger_level = 32768  # midpoint of uint16, auto-initialized after first frame

        # ---- Layout: top bar, plots, bottom bar ----
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vlayout = QtWidgets.QVBoxLayout(central)

        # Top row: readouts + high-level controls
        top_row = QtWidgets.QHBoxLayout()
        self.readout_fs = QtWidgets.QLabel("Fs: — Hz")
        self.readout_peak = QtWidgets.QLabel("Peak: — Hz")
        self.readout_fps = QtWidgets.QLabel("FPS: —")
        self.readout_drops = QtWidgets.QLabel("CRC errors: 0")
        for w in (self.readout_fs, self.readout_peak,
                  self.readout_fps, self.readout_drops):
            w.setStyleSheet("font-family: monospace; font-size: 14px; padding: 4px 12px;")
            top_row.addWidget(w)
        top_row.addStretch()

        self.db_btn = QtWidgets.QPushButton("dB scale")
        self.db_btn.setCheckable(True)
        self.db_btn.toggled.connect(self._on_db_toggled)
        top_row.addWidget(self.db_btn)

        self.trigger_btn = QtWidgets.QPushButton("Trigger")
        self.trigger_btn.setCheckable(True)
        self.trigger_btn.setChecked(True)
        self.trigger_btn.toggled.connect(self._on_trigger_toggled)
        top_row.addWidget(self.trigger_btn)

        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._on_pause)
        top_row.addWidget(self.pause_btn)

        vlayout.addLayout(top_row)

        # Plots row
        plots = pg.GraphicsLayoutWidget()
        vlayout.addWidget(plots, stretch=1)

        self.time_plot = plots.addPlot(title="Waveform")
        self.time_plot.setLabel("bottom", "Time", units="s")
        self.time_plot.setLabel("left", "ADC counts")
        self.time_plot.showGrid(x=True, y=True, alpha=0.3)
        self.time_curve = self.time_plot.plot(pen=pg.mkPen("#00d4aa", width=1))
        self.trigger_line = pg.InfiniteLine(
            angle=0,
            pen=pg.mkPen("#888888", style=QtCore.Qt.PenStyle.DashLine),
            movable=False,
        )
        self.time_plot.addItem(self.trigger_line)

        self.freq_plot = plots.addPlot(title="Magnitude Spectrum")
        self.freq_plot.setLabel("bottom", "Frequency", units="Hz")
        self.freq_plot.setLabel("left", "Magnitude")
        self.freq_plot.showGrid(x=True, y=True, alpha=0.3)
        self.freq_curve = self.freq_plot.plot(pen=pg.mkPen("#ffb040", width=1))
        self.peak_marker = self.freq_plot.plot(
            pen=None, symbol="o", symbolSize=10,
            symbolBrush=pg.mkBrush("#ff5050"),
        )
        self.peak_label = pg.TextItem(
            text="", color="#ff5050", anchor=(0.5, 1.2),
        )
        self.freq_plot.addItem(self.peak_label)
        self.nyquist_line = pg.InfiniteLine(
            angle=90,
            pen=pg.mkPen("#666666", style=QtCore.Qt.PenStyle.DashLine),
            label="Nyquist (Fs/2)",
            labelOpts={"position": 0.9, "color": "#999999"},
        )
        self.freq_plot.addItem(self.nyquist_line)

        # Bottom row: trigger level slider
        bottom_row = QtWidgets.QHBoxLayout()
        bottom_row.addWidget(QtWidgets.QLabel("Trigger level:"))
        self.trigger_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.trigger_slider.setMinimum(0)
        self.trigger_slider.setMaximum(65535)
        self.trigger_slider.setValue(32768)
        self.trigger_slider.valueChanged.connect(self._on_trigger_level_changed)
        bottom_row.addWidget(self.trigger_slider, stretch=1)
        self.trigger_level_label = QtWidgets.QLabel("32768")
        self.trigger_level_label.setStyleSheet("font-family: monospace;")
        bottom_row.addWidget(self.trigger_level_label)
        vlayout.addLayout(bottom_row)

        self.reader = SerialReader(port, self._on_frame_from_reader)
        self._first_frame_rendered = False
        self.reader.start()

        self.timer = QtCore.QTimer()
        self.timer.setInterval(33)
        self.timer.timeout.connect(self._refresh)
        self.timer.start()

    def _on_frame_from_reader(self, frame: Frame):
        with self.latest_frame_lock:
            self.latest_frame = frame

    def _on_pause(self, checked: bool):
        self.paused = checked
        self.pause_btn.setText("Resume" if checked else "Pause")

    def _on_db_toggled(self, checked: bool):
        self.db_scale = checked
        # Re-label y-axis so it's honest about units
        self.freq_plot.setLabel("left", "Magnitude (dB)" if checked else "Magnitude")

    def _on_trigger_toggled(self, checked: bool):
        self.trigger_enabled = checked

    def _on_trigger_level_changed(self, value: int):
        self.trigger_level = value
        self.trigger_level_label.setText(str(value))

    def _refresh(self):
        if self.paused:
            return

        with self.latest_frame_lock:
            frame = self.latest_frame

        if frame is None:
            return

        fs = max(frame.sample_rate_hz, 1)

        # ---- Waveform: apply trigger to align ----
        samples = frame.samples
        
        if self.trigger_enabled:
          edge_idx = find_rising_edge(samples, self.trigger_level)
          if edge_idx > 0:
              # Show only from the edge onward. No pre-trigger history, no padding
              # gymnastics. The visible window shrinks to (N - edge_idx) samples but
              # the user gets a clean, stable waveform that starts AT the trigger.
              samples = samples[edge_idx:]
          # If edge_idx <= 0 (no edge or edge at 0), show raw samples as-is

        t = np.arange(len(samples), dtype=np.float32) / fs        
        y = samples.astype(np.float32) - 32768.0
        self.time_curve.setData(t, y)

        # Move trigger-level horizontal line (centered around 0 after DC-removal)
        self.trigger_line.setValue(self.trigger_level - 32768)

        # ---- Spectrum ----
        freqs = np.arange(MAG_BINS, dtype=np.float32) * fs / (2 * MAG_BINS)
        mags = frame.magnitudes

        if self.db_scale:
            # Avoid log(0); clamp at a tiny floor
            display_mags = 20.0 * np.log10(np.maximum(mags, 1e-6))
        else:
            display_mags = mags

        # Skip bin 0 (DC) for display
        self.freq_curve.setData(freqs[1:], display_mags[1:])

        peak_hz = frame.peak_bin * fs / (2 * MAG_BINS)
        peak_y = display_mags[frame.peak_bin] if frame.peak_bin < MAG_BINS else 0.0
        self.peak_marker.setData([peak_hz], [peak_y])
        # Format peak label nicely: kHz if >= 1 kHz, Hz otherwise
        if peak_hz >= 1000:
            label_text = f"{peak_hz / 1000:.2f} kHz"
        else:
            label_text = f"{peak_hz:.0f} Hz"
        self.peak_label.setText(label_text)
        self.peak_label.setPos(peak_hz, peak_y)

        # Nyquist line at Fs/2
        self.nyquist_line.setValue(fs / 2)

        # Readouts
        self.readout_fs.setText(f"Fs: {fs:>10,} Hz")
        self.readout_peak.setText(f"Peak: {label_text:>12}  (bin {frame.peak_bin})")
        self.readout_drops.setText(f"CRC errors: {self.reader.parser.crc_errors}")

        if not self._first_frame_rendered:
          self.time_plot.enableAutoRange(False)
          self.freq_plot.enableAutoRange(False)
          self._first_frame_rendered = True

        # FPS
        self.displayed_frames += 1
        now = time.monotonic()
        if now - self.last_fps_check >= 1.0:
            self.current_fps = self.displayed_frames / (now - self.last_fps_check)
            self.displayed_frames = 0
            self.last_fps_check = now
            self.readout_fps.setText(f"FPS: {self.current_fps:5.1f}")

    def closeEvent(self, event):
        self.reader.stop()
        super().closeEvent(event)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    w = Viewer(port)
    w.show()
    app.exec()


if __name__ == "__main__":
    main()