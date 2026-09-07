## What is this?

GPS + IMU sensor fusion using a Kalman filter, running on an ESP32 and displayed on a live web map.

GPS updates once a second and drifts a few meters. An IMU updates fast but drifts over time with nothing to correct it. This fuses both — the IMU fills in the gaps between GPS fixes, GPS keeps the estimate from wandering off.

Started with two Jupyter notebooks to get the filter math right before touching hardware: one tracking altitude through a noisy barometer (`[altitude, velocity]` state), one doing 6-state 3D tracking (`[x, y, z, vx, vy, vz]`) with an interceptor that re-targets based on the filter's estimate each step. From there, moved to real hardware: an ESP32 reading an IMU and a GPS module, a complementary filter for roll/pitch/yaw, and a TCP stream to a laptop where the actual 6-state Kalman filter runs in Python and gets plotted live on a Leaflet map.

The filter runs two steps every cycle:

**Predict**, using the motion model:
```
x = F·x + B·u
P = F·P·Fᵀ + Q
```

**Update**, when a new GPS fix comes in:
```
y = z − H·x
S = H·P·Hᵀ + R
K = P·Hᵀ · S⁻¹
x = x + K·y
P = (I − K·H)·P
```

- `x` — state vector (position/velocity)
- `P` — covariance, how uncertain the estimate is
- `F` — motion model, how the state evolves on its own
- `B·u` — control input (IMU acceleration, rotated into the nav frame with gravity removed)
- `Q` — process noise, how much the filter trusts the motion model
- `H` — maps state into what a GPS reading looks like
- `R` — measurement noise, how noisy GPS actually is
- `K` — Kalman gain, how much weight to give the new measurement vs. the prediction

`Predict` runs on every IMU row (~50Hz), `update` only runs when GPS gives a new fix (~1Hz) — predict fast, correct slow. GPS lat/lon gets converted into local East/North/Up meters relative to the first fix (flat-earth approximation, fine at this scale). Initial position uncertainty is set deliberately high so the confidence circle on the map starts big and you can watch it shrink as the filter converges.

---

## Hardware?

- **ESP32** — runs the firmware, reads both sensors, streams data over WiFi (TCP, ~50Hz)
- **IMU (ISM330DHCX, I²C)** — accelerometer + gyroscope, sampled at 104Hz. Gyro bias calibrated on boot (300-sample average). Feeds the complementary filter for roll/pitch/yaw and the acceleration input for the Kalman filter's predict step.
- **GPS (u-blox NEO, UART)** — polled at 1Hz. Gives fix type, satellite count, lat/lon, altitude — this is what the Kalman filter's update step corrects against.
- **OLED (SSD1306, I²C)** — shows fix status, satellite count, and live orientation.
- **microSD (SPI)** — logs every sensor row to CSV, one file per boot.

Each logged row:
```
millis, ax, ay, az, gx, gy, gz, roll, pitch, yaw, fix, sats, lat, lon, alt
1234, 1020, -30, 980, 5, -2, 1, 12.3, -0.5, 91.2, 3, 9, 33.9857, -118.4731, 94.2
```

---

## Setting it up

**Hardware**
1. Flash `main.cpp` to the ESP32 via PlatformIO.
2. Set `WIFI_SSID`/`WIFI_PASS` to match your hotspot.
3. Check Serial Monitor (115200 baud) for the ESP32's IP.

**Python**
```bash
pip install numpy matplotlib
```
Set `ESP_IP` in `telemetry_live_map.py` to the ESP32's IP, then:
```bash
python Telemetry/telemetry_live_map.py
```
This connects to the ESP32 over TCP, runs the Kalman filter on the incoming stream, and writes the fused position to `live_data.json`.

**Web map**
Open `Telemetry/webmap/index.html` with a local server (e.g. VS Code Live Server) — it polls `live_data.json` and renders the live position, confidence circle, and trail on a Leaflet map.

---

## Repo structure

```
telemetry/
├── Kalman Filter Project/
│   └── src/main.cpp              # ESP32 firmware
├── Kalman Filter 2D + 3D Display/
│   ├── kalman filter 2D.ipynb
│   └── kalman filter 3D.ipynb
├── Telemetry/
│   ├── kalman_filter_3d.py       # filter + ENU projection
│   ├── telemetry_live_map.py     # TCP reader + filter loop + JSON writer
│   └── webmap/
│       ├── index.html
│       └── live_data.json
└── Fundamentals of Kalman Filter - Nasa (NOTES)/
```

---

## References

- Welch & Bishop, *An Introduction to the Kalman Filter*, UNC Chapel Hill (1995)
- NASA Fundamentals of Kalman Filtering (in repo notes)
- SparkFun ISM330DHCX Arduino Library
- SparkFun u-blox GNSS Arduino Library
