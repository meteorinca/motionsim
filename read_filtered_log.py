import serial
import time
import re

s = serial.Serial("COM9", 115200, timeout=0.5)
t0 = time.time()
lines = []
while time.time() - t0 < 3.0:
    line = s.readline().decode('utf-8', errors='replace').strip()
    if line and "Fade service not installed" not in line and "ledc_set_duty_and_update" not in line:
        print("LOG:", line)
s.close()
