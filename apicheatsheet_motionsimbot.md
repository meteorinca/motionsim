# MotionSimBot API Cheat Sheet & Control Reference

This document details the REST HTTP API, direct angle commanding, closed-loop PD/PID tuning, automatic E-Stop safety supervisor, time synchronization, and wireless OTA flashing for the **ESP32-S3 MotionSimBot** motion controller.

Base URL: `http://motionsimbot1.local` (or device IP address)

---

## 🌐 Complete HTTP REST API Reference

| Endpoint | Method | Query / Body Parameters | Example | Description |
| :--- | :---: | :--- | :--- | :--- |
| **`/api/angle`** | `GET` | `joint` (1–3, default 1) | `/api/angle?joint=1` | **Read Angle**: Returns JSON with actual angle, target angle, error, raw counts, and duty. |
| **`/api/target`** | `POST` / `GET` | `joint` (1–3), `angle` (degrees) | `/api/target?joint=1&angle=185.0` | **Command Angle**: Smoothly moves the joint to the specified angle under closed loop. |
| **`/api/stats`** | `GET` | — | `/api/stats` | **Full Telemetry**: Returns all 3 joints, PID gains, E-stop state/reason, magnet health, UDP rate, IP, and time sync status. |
| **`/api/arm`** | `POST` | `state` (`1`=Arm, `0`=Disarm) | `/api/arm?state=1` | **Arm/Disarm Motors**: Activates or cuts the IBT-2 driver hardware enable line (GPIO 17). |
| **`/api/estop`** | `POST` | `state` (`1`=Trip, `0`=Clear) | `/api/estop?state=1` | **Software Emergency Stop**: Instantly drops driver enable and all PWM duty cycles to 0. |
| **`/api/estop/clear`**| `POST` | — | `/api/estop/clear` | **Clear E-Stop**: Clears the emergency stop latch and resets runaway/stall hazard counters. |
| **`/api/auto_estop`** | `POST` | `enable` (`1` or `0`) | `/api/auto_estop?enable=1` | **Auto E-Stop Toggle**: Enables/disables automated runaway, stall, and velocity jump monitoring. |
| **`/api/motor/jog`** | `POST` / `GET` | `joint`, `duty` (-819..819), `duration` (ms) | `/api/motor/jog?joint=1&duty=250&duration=400` | **Manual Jog**: Pulses motor duty for commissioning tests (automatically stops after duration). |
| **`/api/motor/invert`**| `POST`| `joint`, `inverted` (`1` or `0`) | `/api/motor/invert?joint=1&inverted=1` | **Invert Polarity**: Reverses motor direction in software (saved to flash NVS). |
| **`/api/autotune`** | `POST` / `GET` | `joint` (1–3) | `/api/autotune?joint=1` | **Autocalibrate PD**: Runs automated step perturbation and calculates optimal Kp & Kd gains. |
| **`/api/pid`** | `GET` | `kp`, `ki`, `kd` | `/api/pid?kp=9.0&kd=1.5` | **Update Gains**: Live updates PID controller stiffness, integral, and damping. |
| **`/api/pid/save`** | `POST` | — | `/api/pid/save` | **Save PID to NVS**: Persists current tuned PID gains across reboots. |
| **`/api/calibrate`** | `POST` | `joint` (1–3) | `/api/calibrate?joint=1` | **Zero Calibration**: Sets current physical angle as 0.00° home reference in NVS. |
| **`/time`** | `GET` | — | `/time` | **Read Time**: Returns current epoch timestamp and NTP sync status (from MyBot). |
| **`/sync_time`** | `GET` | `epoch` (unix seconds) | `/sync_time?epoch=1727045000` | **Sync Time**: Synchronizes internal ESP32 RTC to browser clock. |
| **`/ota`** | `POST` | Binary body (`.bin` file stream) | *(Handled by WebUI)* | **Wireless Firmware Update**: Streams new firmware directly into the next flash OTA partition and reboots. |

---

## 📡 Sample JSON Responses

### 1. `/api/angle?joint=1`
```json
{
  "joint": 1,
  "actual": 182.45,
  "target": 180.00,
  "error": -2.45,
  "duty": -180,
  "raw": 2076
}
```

### 2. `/api/stats`
```json
{
  "m1": { "target": 180.00, "actual": 180.05, "error": -0.05, "duty": -4, "inv": false },
  "m2": { "target": 180.00, "actual": 0.00, "error": 0.00, "duty": 0, "inv": false },
  "m3": { "target": 180.00, "actual": 0.00, "error": 0.00, "duty": 0, "inv": false },
  "pid": { "kp": 8.50, "ki": 0.20, "kd": 1.20 },
  "estop": false,
  "estop_code": 0,
  "estop_reason": "System Normal",
  "auto_estop": true,
  "armed": true,
  "control_mode": 0,
  "autotune": { "running": false, "status": "Idle" },
  "as5600": {
    "detected": true,
    "magnet_ok": true,
    "magnet_too_weak": false,
    "magnet_too_strong": false,
    "agc": 128,
    "magnitude": 2400,
    "raw": 2048,
    "raw_deg": 180.00,
    "cal_deg": 180.00,
    "zero_offset": 0.00
  },
  "time": { "epoch": 1727045120, "synced": true },
  "pkt_rate_hz": 60.0,
  "rx_count": 1240,
  "lost_count": 0,
  "rssi": -52,
  "ip": "192.168.1.145",
  "version": "v1.0-bot"
}
```

---

## 🚨 Multi-Hazard E-Stop Reason Codes

When an emergency stop occurs, `estop_code` and `estop_reason` identify the cause:

| Code | Enumeration | Description |
| :---: | :--- | :--- |
| `0` | `ESTOP_REASON_NONE` | Normal operation. |
| `1` | `ESTOP_REASON_HARDWARE` | Physical emergency stop button pressed on GPIO 18. |
| `2` | `ESTOP_REASON_SOFTWARE` | Software E-stop clicked in WebUI or sent via API. |
| `3` | `ESTOP_REASON_RUNAWAY` | **Auto:** Angle moved away from target while duty applied (e.g., reversed motor wiring). |
| `4` | `ESTOP_REASON_STALL` | **Auto:** High duty (>25%) applied for >1.2 seconds with no actuator movement (jammed). |
| `5` | `ESTOP_REASON_VELOCITY_JUMP` | **Auto:** Angle jump exceeded physical limit (>1500°/s in 10 ms; sensor glitch or magnet slip). |
| `6` | `ESTOP_REASON_MAGNET_FAULT` | **Auto:** AS5600 magnetic status bit dropped (magnet missing, too weak, or detached). |
| `7` | `ESTOP_REASON_LIMIT_EXCEEDED`| **Auto:** Angle exceeded configured software safety limit. |

---

## 🛠️ Step-by-Step Commissioning with cURL

### 1. Check Sensor Health
```bash
curl http://motionsimbot1.local/api/angle?joint=1
```

### 2. Arm the Motors
```bash
curl -X POST http://motionsimbot1.local/api/arm?state=1
```

### 3. Test Motor Jog (25% duty for 400 ms)
```bash
curl -X POST "http://motionsimbot1.local/api/motor/jog?joint=1&duty=250&duration=400"
```

### 4. Invert Motor Polarity (if motor moved opposite to angle increase)
```bash
curl -X POST "http://motionsimbot1.local/api/motor/invert?joint=1&inverted=1"
```

### 5. Command Joint to 185.0°
```bash
curl -X POST "http://motionsimbot1.local/api/target?joint=1&angle=185.0"
```

### 6. Run PD Autocalibration
```bash
curl -X POST http://motionsimbot1.local/api/autotune?joint=1
```

### 7. Save Tuned Gains to Flash
```bash
curl -X POST http://motionsimbot1.local/api/pid/save
```
