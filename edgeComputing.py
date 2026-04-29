import socket
import struct
import csv
import time
from datetime import datetime

# =========================
# CONFIG
# =========================
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# NEW format:
# 2 floats + 2 uint32
fmt = "<2fII"
packet_size = struct.calcsize(fmt)

# =========================
# FILE SETUP
# =========================
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"edge_pf_{timestamp}.csv"

csv_file = open(filename, mode="w", newline="")
writer = csv.writer(csv_file)

writer.writerow([
    "pc_time",
    "roll", "pitch",
    "t_us",
    "proc_time_us"
])

print(f"[INFO] Logging to {filename}")

# =========================
# UDP SOCKET
# =========================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1.0)

print(f"[INFO] Listening on {UDP_IP}:{UDP_PORT}")

# =========================
# LOOP
# =========================
try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)

            if len(data) != packet_size:
                print(f"[WARN] Bad packet size: {len(data)}")
                continue

            roll, pitch, t_us, proc_time = struct.unpack(fmt, data)

            pc_time = time.time()

            writer.writerow([
                pc_time,
                roll, pitch,
                t_us,
                proc_time
            ])

        except socket.timeout:
            continue

except KeyboardInterrupt:
    print("\n[INFO] Stopping logger...")

finally:
    csv_file.close()
    sock.close()
    print(f"[INFO] Saved: {filename}")
