import socket
import threading
import json
import os
import time
import math
from collections import deque

import numpy as np

from kalman_filter_3d import KalmanFilter3D, make_projector3d, body_to_nav_accel

ESP_IP = "172.20.10.8" # replace with whatever main.cpp gives you as the ip address
PORT = 3333
COLS = 15
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webmap")
OUT_FILE = os.path.join(OUT_DIR, "live_data.json")

# how many sigma the drawn circle represents (2.45 ≈ 95% confidence in 2D)
SIGMA_K = 2.45
# start with large uncertainty so the circle visibly NARROWS over time
INITIAL_HORIZ_VAR = 400.0     # m^2  -> 20 m initial std -> big starting circle

rows = deque()
stop_flag = threading.Event()


def reader():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ESP_IP, PORT))
        print("Connected to ESP32 telemetry.")
    except OSError as e:
        print(f"Could not connect to {ESP_IP}:{PORT} -> {e}")
        print("Is the ESP32 powered and is your laptop joined to its network?")
        return
    buf = ""
    while not stop_flag.is_set():
        try:
            data = s.recv(4096)
        except OSError:
            break
        if not data:
            break
        buf += data.decode(errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line or line.startswith("millis"):
                continue
            p = line.split(",")
            if len(p) != COLS:
                continue
            try:
                rows.append({
                    "millis": int(p[0]),
                    "ax": float(p[1]), "ay": float(p[2]), "az": float(p[3]),
                    "roll": float(p[7]), "pitch": float(p[8]), "yaw": float(p[9]),
                    "fix": int(p[10]), "sats": int(p[11]),
                    "lat": float(p[12]), "lon": float(p[13]), "alt": float(p[14]),
                })
            except ValueError:
                continue
    s.close()
    print("Reader stopped.")


kf = KalmanFilter3D(sigma_a=1.5, sigma_gps_horiz=2.5, sigma_gps_vert=5.0)
projector = None
lat0 = lon0 = None
m_per_deg_lon = None
last_millis = None
last_gps = None
last_fix = 0
last_sats = 0
trail = []                      # list of [lat, lon] fused
last_write = 0.0


def enu_to_latlon(e, n):
    return lat0 + n / 110540.0, lon0 + e / m_per_deg_lon


def uncertainty_radius_m():
    """Horizontal 1-sigma from covariance, scaled to the chosen confidence."""
    var = max(kf.P[0, 0], kf.P[1, 1])
    return SIGMA_K * math.sqrt(max(var, 1e-6))


def write_json():
    if lat0 is None or not kf.initialized:
        payload = {"lat": 0, "lon": 0, "radius_m": 0,
                   "fix": last_fix, "sats": last_sats, "trail": []}
    else:
        pe, pn, pu = kf.position
        lat, lon = enu_to_latlon(pe, pn)
        payload = {
            "lat": lat, "lon": lon,
            "radius_m": uncertainty_radius_m(),
            "fix": last_fix, "sats": last_sats,
            "trail": trail[-500:],
        }
    # direct write with a retry — avoids Windows os.replace lock errors
    for _ in range(3):
        try:
            with open(OUT_FILE, "w") as f:
                json.dump(payload, f)
            return
        except PermissionError:
            time.sleep(0.02)   # file briefly locked; wait and retry


def process_available():
    global projector, lat0, lon0, m_per_deg_lon, last_millis, last_gps
    global last_fix, last_sats
    while rows:
        r = rows.popleft()
        last_fix, last_sats = r["fix"], r["sats"]

        if last_millis is not None:
            dt = (r["millis"] - last_millis) / 1000.0
            if 0 < dt < 2.0:
                a_nav = body_to_nav_accel(r["ax"], r["ay"], r["az"],
                                          r["roll"], r["pitch"], r["yaw"])
                kf.predict(dt, a_nav)
        last_millis = r["millis"]

        if r["fix"] >= 2 and (r["lat"] != 0.0 or r["lon"] != 0.0):
            key = (r["lat"], r["lon"], r["alt"])
            if key != last_gps:
                last_gps = key
                if projector is None:
                    lat0, lon0 = r["lat"], r["lon"]
                    m_per_deg_lon = 111320.0 * np.cos(np.radians(lat0))
                    projector = make_projector3d(lat0, lon0)
                e, n, u = projector(r["lat"], r["lon"], r["alt"])
                was_init = kf.initialized
                kf.update(e, n, u)
                if not was_init:
                    # inflate initial horizontal uncertainty -> big starting circle
                    kf.P[0, 0] = INITIAL_HORIZ_VAR
                    kf.P[1, 1] = INITIAL_HORIZ_VAR

        if kf.initialized and lat0 is not None:
            pe, pn, _ = kf.position
            trail.append(list(enu_to_latlon(pe, pn)))


import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation

fus_e_plot, fus_n_plot, fus_u_plot = [], [], []
raw_e_plot, raw_n_plot, raw_u_plot = [], [], []

def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    write_json()
    threading.Thread(target=reader, daemon=True).start()
    print(f"Writing -> {OUT_FILE}")
    print("Open webmap/index.html with VS Code Live Server.")
    print("A matplotlib 3D window will also open.")

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    last_write = [0.0]

    def animate(_):
        process_available()
        # mirror the filter's trail into 3D plot arrays (in meters)
        if kf.initialized:
            pe, pn, pu = kf.position
            fus_e_plot.append(pe); fus_n_plot.append(pn); fus_u_plot.append(pu)

        now = time.time()
        if now - last_write[0] > 0.2:
            write_json()
            last_write[0] = now

        ax.cla()
        ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)'); ax.set_zlabel('Up (m)')
        ax.set_title('IMU + GPS 3D fused path')
        if fus_e_plot:
            ax.plot(fus_e_plot, fus_n_plot, fus_u_plot, 'b-', lw=2, label='fused')
            ax.scatter([fus_e_plot[-1]], [fus_n_plot[-1]], [fus_u_plot[-1]], c='b', s=40)
            ax.legend(loc='upper left')
        return []

    ani = FuncAnimation(fig, animate, interval=200, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop_flag.set()

if __name__ == "__main__":
    main()