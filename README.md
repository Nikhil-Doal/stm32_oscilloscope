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