# Kalman Filter Navigation System
**UCLA EE/CE | Defense Portfolio Build**

Python simulation of discrete-time Kalman Filters for 1D altitude tracking and 3D ballistic trajectory estimation with active interception. Part of a larger IMU + GPS sensor fusion project targeting avionics and inertial navigation applications.

---

## Simulations

### `kalman filter 2D.ipynb` — Altitude Tracking
Simulates a barometric altimeter tracking a rocket's vertical flight profile through noisy sensor readings.

- **State vector:** `[altitude, velocity]`
- **Sensor model:** Gaussian noise (σ = 5.0 m) on true altitude
- **Matrices:** Standard `F`, `H`, `Q`, `R` with Joseph-form covariance update for numerical stability
- **Output:** Animated plot of true altitude vs. raw barometer readings vs. Kalman estimate converging over 80 timesteps

### `kalman filter 3D.ipynb` — Ballistic Trajectory + Interception
Simulates a rocket on a ballistic arc and an interceptor guided by Kalman Filter position estimates in real time.

- **State vector:** `[x, y, z, vx, vy, vz]` (6-state)
- **Sensor model:** Radar noise (σ = 2.0 m) on 3D position
- **Gravity compensation:** Applied directly to the z-velocity state during the predict step
- **Interceptor logic:** Re-targets toward KF estimate each timestep; intercept declared at < 5 m proximity
- **Output:** Animated dual-panel plot — 3D trajectory manifold (top) and 2D bird's-eye view with Kalman overlay (bottom)

---

## Math Overview

Each filter runs the standard predict → update loop:

**Predict**
```
x = F @ x
P = F @ P @ F.T + Q
```

**Update**
```
y = z - H @ x          # residual
S = H @ P @ H.T + R    # innovation covariance
K = P @ H.T @ inv(S)   # Kalman gain
x = x + K @ y
P = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T   # Joseph form
```

`Q` controls how much the filter trusts the motion model. `R` controls how much it trusts the sensor. Both were tuned empirically against simulated sensor noise floors.

---

## Setup

```bash
pip install numpy matplotlib ipympl
```

Run notebooks in JupyterLab or VS Code with the Jupyter extension. The 3D notebook requires interactive input at startup (launch point A, target point B, interceptor start C).

---

## Repo Structure

```
Kalman-Filter-Project/
└── Kalmann Filter Project/
    ├── kalman filter 2D.ipynb   # altitude tracking simulation
    └── kalman filter 3D.ipynb   # 3D ballistic intercept simulation
```

---

## Roadmap

This simulation work is Phase 1 of a full IMU + GPS navigation system:

| Phase | Goal |
|-------|------|
| **1 — Done** | Python KF simulation (altitude + 3D trajectory) |
| **2** | Hardware bring-up: ICM-42688-P IMU via SPI + u-blox NEO-M9N GPS via UART on ESP32 |
| **3** | Port KF to embedded C; fuse live IMU + GPS at 500 Hz predict / async GPS update |
| **4** | Custom PCB: ESP32 + IMU + GPS + microSD + RFM95W LoRa in KiCad |
| **5** | Outdoor walk test, fused vs. raw GPS comparison, demo video |

Target hardware: **ICM-42688-P** (higher precision than MPU-6050, closer to defense-grade specs) + **u-blox NEO-M9N** (10 Hz update rate, significantly better accuracy than NEO-6M).

---

## Background

Kalman filters are the backbone of inertial navigation in avionics, drone autonomy, and GPS-denied environments — active research priorities at Northrop Grumman, Raytheon, JPL, and Anduril. This project directly extends rocketry EGSE sensor work and applies Linear Algebra and Differential Equations coursework to real hardware.
