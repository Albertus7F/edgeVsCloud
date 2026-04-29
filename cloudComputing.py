import socket
import struct
import time
import csv
import numpy as np
from datetime import datetime

# =========================
# CONFIG
# =========================
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

fmt = "<6fI"  # ax ay az gx gy gz t_us
packet_size = struct.calcsize(fmt)

N = 1000  # use same as ESP test (or 1000 later)

# =========================
# PARTICLE FILTER
# =========================
particles = np.zeros((N, 2))  # roll, pitch
weights = np.ones(N) / N

def predict(gx, gy, dt):
    noise = 0.002
    particles[:,0] += gx * dt + np.random.randn(N) * noise
    particles[:,1] += gy * dt + np.random.randn(N) * noise

def update(ax, ay, az):
    norm = np.sqrt(ax*ax + ay*ay + az*az)
    if norm < 1e-6:
        return

    axn, ayn, azn = ax/norm, ay/norm, az/norm

    global weights
    for i in range(N):
        r, p = particles[i]

        gx = -np.sin(p)
        gy = np.sin(r)*np.cos(p)
        gz = np.cos(r)*np.cos(p)

        err = (axn-gx)**2 + (ayn-gy)**2 + (azn-gz)**2
        weights[i] = np.exp(-err * 5.0)

    s = np.sum(weights)
    if s < 1e-12:
        weights[:] = 1.0/N
    else:
        weights /= s

def resample():
    global particles, weights

    idx = np.random.choice(N, N, p=weights)
    particles = particles[idx]
    weights[:] = 1.0/N

def estimate():
    r = np.sum(particles[:,0] * weights)
    p = np.sum(particles[:,1] * weights)
    return r, p

# =========================
# CSV SETUP
# =========================
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"cloud_pf_{timestamp}.csv"

csv_file = open(filename, "w", newline="")
writer = csv.writer(csv_file)

writer.writerow([
    "pc_time",
    "ax","ay","az",
    "gx","gy","gz",
    "roll","pitch",
    "t_us",
    "proc_time_us"
])

print("[INFO] Logging to", filename)

# =========================
# SOCKET
# =========================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1.0)

last_t = None

# =========================
# LOOP
# =========================
try:
    while True:
        try:
            data, _ = sock.recvfrom(1024)

            if len(data) != packet_size:
                continue

            ax, ay, az, gx, gy, gz, t_us = struct.unpack(fmt, data)

            # compute dt from ESP time
            if last_t is None:
                last_t = t_us
                continue

            dt = (t_us - last_t) / 1e6
            last_t = t_us

            # =========================
            # PF PROCESSING TIME
            # =========================
            t0 = time.time()

            predict(gx, gy, dt)
            update(ax, ay, az)
            resample()
            roll, pitch = estimate()

            t1 = time.time()
            proc_time = (t1 - t0) * 1e6  # microseconds
            # =========================

            writer.writerow([
                time.time(),
                ax, ay, az,
                gx, gy, gz,
                roll, pitch,
                t_us,
                proc_time
            ])

        except socket.timeout:
            continue

except KeyboardInterrupt:
    print("\n[INFO] Stopping...")

finally:
    csv_file.close()
    sock.close()
    print("[INFO] Saved:", filename)
