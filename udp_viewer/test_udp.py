import socket
import struct
import time

# Create 66 floats for extradata=3 (264 bytes)
floats = [0.0] * 66

# Float 0: time
floats[0] = 45.2
# Float 1: lap_time
floats[1] = 45.2
# Float 2: lap_distance
floats[2] = 2500.0
# Float 3: total_distance
floats[3] = 2500.0
# Float 7: speed (m/s) -> 35 m/s = 126 km/h
floats[7] = 35.0
# Float 14: pitch
floats[14] = 0.05
# Float 15: roll
floats[15] = -0.02
# Float 29: throttle
floats[29] = 0.85
# Float 30: steer
floats[30] = 0.12
# Float 31: brake
floats[31] = 0.0
# Float 33: gear (e.g. 4 -> gear 3)
floats[33] = 4.0
# Float 34: gLat
floats[34] = 0.82
# Float 37: rpm (div by 10) -> 5400 RPM
floats[37] = 540.0
# Float 61: track_length
floats[61] = 10000.0

packet = struct.pack("<66f", *floats)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(packet, ("127.0.0.1", 20777))
print("Sent test DiRT Rally 2.0 UDP packet (264 bytes) to 127.0.0.1:20777")
