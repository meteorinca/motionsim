import serial
import time

s = serial.Serial("COM9", 115200, timeout=0.1)
s.dtr = False
s.rts = False
time.sleep(0.1)
s.dtr = True
s.rts = True
time.sleep(0.1)

print("Listening for 3 seconds...")
t0 = time.time()
buf = ""
while time.time() - t0 < 3.0:
    chunk = s.read(s.in_waiting or 1).decode('utf-8', errors='replace')
    if chunk:
        buf += chunk
s.close()
print("Output length:", len(buf))
print(buf[:1000])
