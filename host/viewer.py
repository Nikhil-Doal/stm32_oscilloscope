# parsing
import struct
import sys
import time
import threading
from collections import deque
from dataclasses import dataclass
# plotting
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
PAYLOAD_BYTES = PAYLOAD_HDR_BYTES + WAVEFORM_SAMPLES * 2 + MAG_BINS * 4  # 6160
FRAME_OVERHEAD = 7  # 2 sync + 1 type + 2 len + 2 crc
FRAME_TOTAL = FRAME_OVERHEAD + PAYLOAD_BYTES  # 6167

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
    samples: np.ndarray       # uint16 shape (2048,)
    magnitudes: np.ndarray    # float32 shape (512,)


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
        # Find sync bytes
        while len(self.buf) >= 2:
            if self.buf[0] == SYNC_0 and self.buf[1] == SYNC_1:
                break
            # Discard one byte and try again
            del self.buf[0]

        # Need full frame
        if len(self.buf) < FRAME_TOTAL:
            return None

        # Extract length and sanity check
        payload_len = self.buf[3] | (self.buf[4] << 8)
        if payload_len != PAYLOAD_BYTES:
            # Length mismatch - sync was false positive, skip one byte and retry
            del self.buf[0]
            return None

        # Validate CRC (over type + length + payload, NOT sync)
        crc_region = bytes(self.buf[2:2 + 3 + PAYLOAD_BYTES])
        crc_received = self.buf[FRAME_TOTAL - 2] | (self.buf[FRAME_TOTAL - 1] << 8)
        crc_expected = crc16_ccitt(crc_region)

        if crc_received != crc_expected:
            self.crc_errors += 1
            del self.buf[0]
            return None

        # Parse payload
        off = 5  # skip sync + type + len
        tick, fs, peak_bin = struct.unpack_from("<III", self.buf, off)
        peak_mag, = struct.unpack_from("<f", self.buf, off + 12)

        samples_off = off + PAYLOAD_HDR_BYTES
        samples = np.frombuffer(
            self.buf, dtype=np.uint16,
            count=WAVEFORM_SAMPLES,
            offset=samples_off,
        ).copy()  # copy so we can discard buf

        mags_off = samples_off + WAVEFORM_SAMPLES * 2
        mags = np.frombuffer(
            self.buf, dtype=np.float32,
            count=MAG_BINS,
            offset=mags_off,
        ).copy()

        # Consume frame
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


# --- GUI --------------------------------------------------------------------

class Viewer(QtWidgets.QMainWindow):
    def __init__(self, port: str):
        super().__init__()
        self.setWindowTitle("STM32H7 Oscilloscope")
        self.resize(1400, 700)

        # Latest frame (set by reader thread, read by GUI timer)
        self.latest_frame = None
        self.latest_frame_lock = threading.Lock()

        # FPS tracking
        self.displayed_frames = 0
        self.last_fps_check = time.monotonic()
        self.current_fps = 0.0
        self.paused = False

        # Central layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vlayout = QtWidgets.QVBoxLayout(central)

        # Top row: readouts + controls
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

        self.freq_plot = plots.addPlot(title="Magnitude Spectrum")
        self.freq_plot.setLabel("bottom", "Frequency", units="Hz")
        self.freq_plot.setLabel("left", "Magnitude")
        self.freq_plot.showGrid(x=True, y=True, alpha=0.3)
        self.freq_curve = self.freq_plot.plot(pen=pg.mkPen("#ffb040", width=1))
        self.peak_marker = self.freq_plot.plot(
            pen=None, symbol="o", symbolSize=10,
            symbolBrush=pg.mkBrush("#ff5050"),
        )

        # Start serial reader
        self.reader = SerialReader(port, self._on_frame_from_reader)
        self.reader.start()

        # GUI refresh timer (~30 Hz)
        self.timer = QtCore.QTimer()
        self.timer.setInterval(33)
        self.timer.timeout.connect(self._refresh)
        self.timer.start()

    # Called from the reader thread — keep it minimal, no Qt widget touches
    def _on_frame_from_reader(self, frame: Frame):
        with self.latest_frame_lock:
            self.latest_frame = frame

    def _on_pause(self, checked: bool):
        self.paused = checked
        self.pause_btn.setText("Resume" if checked else "Pause")

    def _refresh(self):
        if self.paused:
            return

        with self.latest_frame_lock:
            frame = self.latest_frame

        if frame is None:
            return

        # Time-domain: x axis in seconds, samples centered at 0
        fs = max(frame.sample_rate_hz, 1)
        t = np.arange(WAVEFORM_SAMPLES, dtype=np.float32) / fs
        y = frame.samples.astype(np.float32) - 32768.0
        self.time_curve.setData(t, y)

        # Frequency-domain: x axis in Hz
        freqs = np.arange(MAG_BINS, dtype=np.float32) * fs / (2 * MAG_BINS)
        # Skip bin 0 (DC) when plotting so the spectrum doesn't get crushed
        self.freq_curve.setData(freqs[1:], frame.magnitudes[1:])

        peak_hz = frame.peak_bin * fs / (2 * MAG_BINS)
        self.peak_marker.setData([peak_hz], [frame.peak_mag])

        # Readouts
        self.readout_fs.setText(f"Fs: {fs:>10,} Hz")
        self.readout_peak.setText(f"Peak: {peak_hz:>8,.0f} Hz  (bin {frame.peak_bin})")
        self.readout_drops.setText(f"CRC errors: {self.reader.parser.crc_errors}")

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