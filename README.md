# STM32H7 Real-Time Oscilloscope

Real-time digital oscilloscope on STM32H753ZI. 1 MSa/s ADC sampling via DMA with cache-coherent buffers, FreeRTOS task-based pipeline, CMSIS-DSP FFT, USB CDC streaming to a Python pyqtgraph viewer.

## Structure

- `firmware/` — STM32CubeIDE project (C, FreeRTOS, CMSIS-DSP)
- `host/` — Python viewer (pyqtgraph)
- `docs/` — design notes, benchmarks, block diagram

## Hardware

- NUCLEO-H753ZI
- Analog input on PA0 (0–3.3 V)
- Output: USB CDC over the Nucleo's USB connector


STM32H7 Oscilloscope Viewer

Connects to the USB CDC virtual COM port from the H753ZI firmware,
parses 6167-byte binary frames containing raw ADC samples and FFT magnitudes,
and plots them live with pyqtgraph.

Frame layout (little-endian):
  [0xAA 0x55]                         sync
  [type:u8=0x01]                      frame type
  [payload_len:u16]                   6160
  [tick:u32]                          MCU tick in ms
  [sample_rate:u32]                   Hz
  [peak_bin:u32]                      index into magnitudes
  [peak_mag:f32]                      magnitude at peak
  [samples:u16 x 2048]                raw 16-bit ADC counts
  [magnitudes:f32 x 512]              |FFT| per bin
  [crc16_ccitt:u16]                   over [type..magnitudes]


Byte-streaming parser. Keeps an internal buffer, walks it looking for
sync, validates length and CRC, yields Frame objects.

Designed to be robust to partial reads and mid-stream reconnection:
if CRC fails or a sync search fails, it discards just enough bytes
to resume finding the next valid frame.