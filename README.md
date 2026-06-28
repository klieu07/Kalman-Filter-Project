# Kalman Filter Navigation System
**UCLA EE/CE | Defense Portfolio Build**

A full-stack inertial navigation system spanning Python simulation, embedded C firmware, and a live web map — built around Kalman Filter sensor fusion of IMU accelerometer data with GPS position fixes.

---

## What Is a Kalman Filter?

A Kalman Filter is an optimal recursive estimator. It answers one question: given a noisy model of how a system moves and noisy sensor measurements of where it actually is, what is the best possible estimate of the true state?

Every iteration runs two steps:

**Predict** — use the motion model to project the state forward in time:
```
x = F·x + B·u        # propagate state (position, velocity) + apply control input (acceleration)
P = F·P·Fᵀ + Q       # covariance grows as uncertainty accumulates
```

**Update** — correct the prediction using a new sensor measurement:
```
y = z − H·x          # residual: how far off is the prediction from the measurement?
S = H·P·Hᵀ + R       # innovation covariance: total uncertainty in the residual
K = P·Hᵀ · S⁻¹       # Kalman gain: how much to trust the measurement vs. the model
x = x + K·y          # fused estimate
P = (I − K·H)·P      # covariance shrinks as the sensor anchors the estimate
```

Two noise matrices govern the filter's personality:
- **Q (process noise)** — how much the filter trusts the motion model. Higher Q = follows sensor more aggressively.
- **R (measurement noise)** — how noisy the sensor is. Higher R = trusts the model more, filters out noise harder.

The Kalman Filter is provably optimal (minimum mean squared error) under the assumption of Gaussian noise and linear dynamics. It is the backbone of every modern inertial navigation system — GPS receivers, aircraft INS, missile guidance, autonomous vehicles, and spacecraft attitude control all run variants of this algorithm.

---

## System Architecture

```
┌──────────────────────────┐      TCP/WiFi        ┌───────────────────────────┐
│       ESP32 Hardware     │  ─────────────────►  │     Python Telemetry      │
│                          │   CSV stream, ~50 Hz │                           │
│  ISM330DHCX IMU          │                      │  KalmanFilter3D           │
│  u-blox GPS (1 Hz)       │                      │  ├── predict() @ IMU rate │
│  Complementary Filter    │                      │  └── update()  @ GPS rate │
│  SD card logger          │                      │                           │
│  OLED display            │                      │  write live_data.json     │
│  WiFi TCP server         │                      │  matplotlib 3D plot       │
└──────────────────────────┘                      └────────────┬──────────────┘
                                                               │  file poll (200 ms)
                                                  ┌────────────▼──────────────┐
                                                  │     Leaflet.js Web Map    │
                                                  │                           │
                                                  │  live position marker     │
                                                  │  uncertainty circle       │
                                                  │  fused path trail         │
                                                  │  HUD: fix / sats / coords │
                                                  └───────────────────────────┘
```

---

## Phase 1 — Simulation: `Kalman Filter 2D + 3D Display/`

Before any hardware was touched, the filter math was validated in two Python notebooks.

### `kalman filter 2D.ipynb` — Altitude Tracking

Simulates a barometric altimeter tracking a rocket's vertical flight through noisy sensor readings.

- **State vector:** `[altitude, velocity]`
- **Sensor model:** Gaussian noise (σ = 5.0 m) on true altitude
- **Output:** Animated plot — true altitude vs. raw barometer vs. Kalman estimate converging over 80 timesteps

### `kalman filter 3D.ipynb` — Ballistic Trajectory + Interception

Simulates a rocket on a ballistic arc and an interceptor guided by real-time Kalman Filter position estimates.

- **State vector:** `[x, y, z, vx, vy, vz]` (6-state)
- **Sensor model:** Radar noise (σ = 2.0 m) on 3D position
- **Gravity compensation:** Applied directly to the z-velocity state during the predict step
- **Interceptor logic:** Re-targets toward KF estimate each timestep; intercept declared at < 5 m proximity
- **Output:** Animated dual-panel — 3D trajectory (top) and 2D bird's-eye view with Kalman overlay (bottom)

Both notebooks use the Joseph-form covariance update for numerical stability:
```
P = (I − K·H)·P·(I − K·H)ᵀ + K·R·Kᵀ
```

---

## Phase 2 — Embedded Firmware: `Kalman Filter Project/src/main.cpp`

The ESP32 firmware runs as the data acquisition layer. It reads two sensors simultaneously, fuses them into attitude angles, and streams everything over WiFi.

### Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| Microcontroller | ESP32 | — |
| IMU | SparkFun ISM330DHCX | I²C |
| GPS | u-blox NEO (GNSS library) | UART (Serial2, pins 16/17) |
| Display | SSD1306 OLED 128×64 | I²C (0x3C) |
| Storage | microSD | SPI (HSPI: SCK=14, MISO=27, MOSI=13, CS=33) |

### What the Firmware Does

**IMU Setup**
The ISM330DHCX is configured at 104 Hz output data rate, ±4g accelerometer range, ±500 dps gyroscope range, with hardware low-pass filters on both axes. On boot, 300 gyro samples are averaged to compute static bias offsets that are subtracted from every subsequent reading.

**Attitude Estimation (Complementary Filter)**
A complementary filter fuses the gyroscope integration (fast, drifts) with the accelerometer gravity vector (slow, noisy) to maintain roll and pitch. Yaw is integrated from the gyro alone:

```
roll  = α·(roll  + gx·dt) + (1−α)·roll_acc
pitch = α·(pitch + gy·dt) + (1−α)·pitch_acc
yaw  += gz·dt
```

`α = 0.98` — 98% weight on the gyro integration for short-term accuracy, 2% on the accelerometer for long-term drift correction.

**GPS**
The u-blox module is polled at 1 Hz via UBX protocol over UART at 38400 baud. Fix type (2D/3D), satellite count, latitude, longitude, and altitude (MSL) are cached between reads.

**SD Card Logging**
Each boot opens a new sequentially named log file (`log000.csv`, `log001.csv`, ...). Every sensor row is written and flushed to disk every second. The row format is:
```
millis,ax,ay,az,gx,gy,gz,roll,pitch,yaw,fix,sats,lat,lon,alt
```

**OLED Display**
The 128×64 screen shows GPS fix status, satellite count, live roll/pitch/yaw angles, SD log row count, and a WiFi connection indicator. An isometric 3D orientation tripod is drawn in real time using a ZYX rotation matrix, showing the physical attitude of the board.

**WiFi TCP Server**
The ESP32 joins a mobile hotspot (SSID + password in firmware) and starts a TCP server on port 3333. When the laptop connects, it receives the same CSV header followed by a continuous stream of sensor rows at ~50 Hz. The ESP32's IP address is printed to Serial on boot.

---

## Phase 3 — Python Telemetry: `Telemetry/`

### `kalman_filter_3d.py` — The Filter Core

Implements the full 6-state Kalman Filter for real hardware data.

**State vector:** `[E, N, U, vE, vN, vU]` — position and velocity in East/North/Up coordinates relative to the first GPS fix.

**`make_projector3d(lat0, lon0)`**
Converts raw GPS coordinates into local ENU meters using a flat-Earth tangent plane approximation centered on the first fix:
```
E = (lon − lon0) · (111320 · cos(lat0°))
N = (lat − lat0) · 110540
U = altitude (meters MSL)
```

**`body_to_nav_accel(ax, ay, az, roll, pitch, yaw)`**
Rotates the IMU's body-frame accelerometer reading (in mg) into the navigation frame and removes gravity from the Up axis:
```
a_body  = [ax, ay, az] · (G / 1000)     # mg → m/s²
R_body→nav = ZYX rotation matrix (roll, pitch, yaw)
a_nav   = R · a_body
a_nav[U] -= G                            # subtract gravity
```
This is the control input `u` that drives the predict step, turning raw IMU readings into estimated linear acceleration in real-world East/North/Up axes.

**`KalmanFilter3D`**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `sigma_a` | 1.5 m/s² | Assumed process acceleration noise |
| `sigma_gps_horiz` | 2.5 m | Horizontal GPS noise (1-sigma) |
| `sigma_gps_vert` | 5.0 m | Vertical GPS noise (1-sigma) |

The process noise matrix Q is built from a random-acceleration model:
```
Q_pos-pos = (dt⁴/4) · σ_a²
Q_pos-vel = (dt³/2) · σ_a²
Q_vel-vel = (dt²)   · σ_a²
```

`predict(dt, a_nav)` runs at IMU rate (every row). `update(E, N, U)` runs only when a new unique GPS fix arrives. This predict-fast / update-slow architecture is what lets the filter track smooth motion between slow GPS updates.

On first GPS fix, horizontal position variance is inflated to 400 m² (20 m initial std), so the confidence circle on the map starts large and visibly shrinks as the filter converges.

---

### `telemetry_live_map.py` — Data Pipeline

This script ties everything together. It runs two concurrent threads:

**Reader thread** opens a TCP socket to the ESP32 IP, receives the CSV stream, parses each row, and pushes parsed dicts into a `deque`.

**Main thread / animation loop** runs at 200 ms intervals:
1. Drains the deque — calls `predict()` on every IMU row (using `dt` from the millisecond timestamp delta), and `update()` whenever a new GPS coordinate appears.
2. Converts the fused ENU position back to lat/lon via the inverse of the ENU projection.
3. Appends the fused position to a trail buffer (capped at 500 points).
4. Computes the current 95% confidence radius from the Kalman covariance diagonal: `r = 2.45 · sqrt(max(P[0,0], P[1,1]))`.
5. Writes `live_data.json` to `webmap/` with the current position, radius, fix status, satellite count, and trail.
6. Updates the matplotlib 3D window with the fused 3D path in ENU meters.

**Data flow through the hotspot:**
```
Phone hotspot → ESP32 joins network → ESP32 IP printed to Serial
→ ESP_IP set in telemetry_live_map.py → Python connects via TCP socket
→ CSV rows stream at ~50 Hz over WiFi → Python decodes + filters in real time
```

---

## Phase 4 — Live Web Map: `Telemetry/webmap/index.html`

A single-file browser application built on Leaflet.js that polls `live_data.json` every 200 ms and renders the fused position live.

### What Renders on the Map

| Element | Source | Description |
|---------|--------|-------------|
| Blue dot | `lat`, `lon` | Current fused position |
| Shrinking circle | `radius_m` | 95% horizontal confidence ellipse from KF covariance |
| Blue trail | `trail[]` | Last 500 fused positions — the path traveled |
| HUD overlay | `fix`, `sats`, `radius_m`, `lat`, `lon` | Live status readout |

**Map layers:** OpenStreetMap street view and Esri World Imagery satellite, switchable via a layer control. On the first GPS fix, the map auto-centers at zoom level 17. After that the user can freely pan and zoom.

**Uncertainty circle behavior:** Starts large (radius > 20 m) on first fix because the Kalman covariance is initialized high. As the filter processes more GPS updates and IMU predictions, covariance contracts and the circle shrinks in real time — visually demonstrating filter convergence.

The page uses a `?t=Date.now()` cache-buster on every fetch to guarantee it always reads the freshest JSON rather than a browser-cached version.

---

## Data Format

Every row from the ESP32:
```
millis,  ax,  ay,  az,  gx,  gy,  gz,  roll, pitch, yaw, fix, sats,    lat,       lon,     alt
1234,  1020, -30, 980,   5,  -2,   1, 12.3,  -0.5, 91.2,   3,    9, 33.9857, -118.4731, 94.2
```

| Field | Unit | Description |
|-------|------|-------------|
| `millis` | ms | ESP32 uptime |
| `ax/ay/az` | mg | Accelerometer (raw, ±4g range) |
| `gx/gy/gz` | mdps | Gyroscope (raw, ±500 dps range) |
| `roll/pitch/yaw` | degrees | Complementary filter attitude |
| `fix` | 0–3 | GPS fix type (0=none, 2=2D, 3=3D) |
| `sats` | count | Satellites in view |
| `lat/lon` | decimal degrees | GPS position (7 decimal places) |
| `alt` | meters MSL | GPS altitude |

`live_data.json` (written by Python, read by browser):
```json
{
  "lat": 33.985712,
  "lon": -118.473108,
  "radius_m": 6.3,
  "fix": 3,
  "sats": 9,
  "trail": [[33.9857, -118.4731], ...]
}
```

---

## Setup & Running

### Hardware
1. Flash `Kalman Filter Project/src/main.cpp` to the ESP32 via PlatformIO.
2. Set `WIFI_SSID` and `WIFI_PASS` in `main.cpp` to match your phone's mobile hotspot.
3. Open Serial Monitor (115200 baud) — note the ESP32 IP address printed on boot.
4. Connect your laptop to the same hotspot.

### Python
```bash
pip install numpy matplotlib
```

Set `ESP_IP` in `telemetry_live_map.py` to the ESP32's IP, then:
```bash
python Telemetry/telemetry_live_map.py
```

### Web Map
Open `Telemetry/webmap/index.html` with VS Code Live Server (or any local HTTP server). The map updates live as `live_data.json` is rewritten.

---

## Repo Structure

```
telemetry/
├── Kalman Filter Project/
│   ├── platformio.ini
│   └── src/
│       └── main.cpp                  # ESP32 firmware
├── Kalman Filter 2D + 3D Display/
│   ├── kalman filter 2D.ipynb        # altitude tracking simulation
│   └── kalman filter 3D.ipynb        # 3D ballistic intercept simulation
├── Telemetry/
│   ├── kalman_filter_3d.py           # KF math + ENU projection + body→nav rotation
│   ├── telemetry_live_map.py         # TCP reader, filter loop, JSON writer, 3D plot
│   └── webmap/
│       ├── index.html                # Leaflet live map
│       └── live_data.json            # filter output (rewritten at 5 Hz)
└── Fundamentals of Kalman Filter - Nasa (NOTES)/
    └── *.jpg                         # NASA KF reference material
```

---

## Why This Matters

GPS alone updates at 1 Hz and has ~2–5 m horizontal error under open sky, degrading further in canyons, under tree cover, or indoors. IMU alone accumulates drift that grows unbounded without correction. The Kalman Filter fuses both: the IMU provides high-rate smooth motion prediction between slow GPS fixes, while GPS anchors the estimate and prevents drift from accumulating. The result is a position estimate that is smoother, more responsive, and more accurate than either sensor alone.

This is the exact architecture behind every serious navigation system — commercial aviation INS, military guided munitions, autonomous vehicle localization, and spacecraft attitude determination all trace directly back to this predict-update loop. The specific stack here (ESP32 + IMU + GPS + KF + live map) demonstrates the full signal chain: from raw MEMS sensor bits to a calibrated, filtered, georeferenced position rendered on a live map over a wireless link — the same system, scaled up, that navigates a cruise missile or lands a rocket.

The covariance visualization (the shrinking circle) is also non-trivial: it shows the filter's self-assessed confidence, not just position. Watching it converge in real time demonstrates that the system understands its own uncertainty — a property fundamental to safe autonomous systems where you need to know not just where you think you are, but how confident you are in that answer.

---

## References

- Welch & Bishop, *An Introduction to the Kalman Filter*, UNC Chapel Hill (1995)
- NASA Fundamentals of Kalman Filtering (included in repo notes)
- SparkFun ISM330DHCX Arduino Library
- SparkFun u-blox GNSS Arduino Library
- Leaflet.js 1.9.4
