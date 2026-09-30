import serial
import time

try:
    s = serial.Serial("COM9", 115200, timeout=1.5)
    time.sleep(1.0)
    data = s.read(s.in_waiting or 500)
    print("--- SERIAL RECEIVED ---")
    print(data.decode("utf-8", errors="replace"))
    s.close()
except Exception as e:
    print(f"Error: {e}")
