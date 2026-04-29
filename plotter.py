import pandas as pd
import matplotlib.pyplot as plt

# =========================
# FILES (CHANGE THESE)
# =========================
edge_file = "edge_pf_20260429_101640.csv"
cloud_file = "cloud_pf_20260429_101931.csv"

# =========================
# LOAD DATA
# =========================
edge = pd.read_csv(edge_file)
cloud = pd.read_csv(cloud_file)

# =========================
# NORMALIZE TIME (start at 0)
# =========================
edge["t"] = edge["pc_time"] - edge["pc_time"].iloc[0]
cloud["t"] = cloud["pc_time"] - cloud["pc_time"].iloc[0]

# =========================
# TAKE FIRST 60 SECONDS
# =========================
edge_1min = edge[edge["t"] <= 60]
cloud_1min = cloud[cloud["t"] <= 60]

# =========================
# COMPUTE AVERAGE PROCESS TIME
# =========================
edge_avg = edge_1min["proc_time_us"].mean()
cloud_avg = cloud_1min["proc_time_us"].mean()

print("===== PROCESSING TIME =====")
print(f"Edge (ESP32 PF):   {edge_avg:.2f} us")
print(f"Cloud (Pi PF):     {cloud_avg:.2f} us")

# =========================
# PLOT ROLL
# =========================
plt.figure()
plt.plot(edge_1min["t"], edge_1min["roll"], label="Edge Roll")
plt.plot(cloud_1min["t"], cloud_1min["roll"], label="Cloud Roll")
plt.xlabel("Time (s)")
plt.ylabel("Roll (rad)")
plt.title("Roll Comparison (First 60s)")
plt.legend()
plt.grid()

# =========================
# PLOT PITCH
# =========================
plt.figure()
plt.plot(edge_1min["t"], edge_1min["pitch"], label="Edge Pitch")
plt.plot(cloud_1min["t"], cloud_1min["pitch"], label="Cloud Pitch")
plt.xlabel("Time (s)")
plt.ylabel("Pitch (rad)")
plt.title("Pitch Comparison (First 60s)")
plt.legend()
plt.grid()

plt.show()
