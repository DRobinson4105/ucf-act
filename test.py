#!/usr/bin/env python3
import serial
import time
import sys

PORT = "/dev/ttyACM0"
BAUD = 1_000_000
SYNC = 0xAA

throttle = 0
steering = 180
braking  = 3

port = serial.Serial(PORT, BAUD, timeout=1)
print(f"Sending on {PORT} at {BAUD} baud. Ctrl+C to stop.")

seq = 0
try:
    while True:
        seq = (seq + 1) & 0xFF
        frame = bytes([
            SYNC,
            seq,
            (throttle >> 8) & 0xFF,
            throttle & 0xFF,
            (steering >> 8) & 0xFF,
            steering & 0xFF,
            braking & 0xFF,
        ])
        port.write(frame)
        print(f"  seq={seq} thr={throttle} steer={steering} brake={braking}")
        time.sleep(0.1)   # 10 Hz
except KeyboardInterrupt:
    port.close()
    print("Stopped.")
