import serial
import time

PORT = "COM3"   # adjust if yours is different

with serial.Serial(PORT, timeout=0.1) as ser:
    print(f"opened {PORT}, reading for 3s...")
    t0 = time.time()
    bytes_read = 0
    while time.time() - t0 < 3.0:
        chunk = ser.read(4096)
        bytes_read += len(chunk)
    print(f"read {bytes_read} bytes in 3s = {bytes_read/3/1024:.1f} KB/s")