# MotionSimBot API Cheat Sheet & Hardware Wiring Guide

This document lists the hardware wiring, 12-bit Hall angle sensor math, IBT-2 motor driver specifications, REST HTTP API endpoints, and closed-loop PID control guidelines for the `motionsimbot` production firmware.

---

## 🔌 Hardware Wiring Guide (ESP32-C3 MotionSimBot)

### 1. IBT-2 Motor Drivers (3 Motors: Roll, Pitch, Heave)

Each motor is controlled by an **IBT-2 (BTS7960)** H-bridge module connected to 24V 250W motors:

| Motor | Function | RPWM (Forward) | LPWM (Reverse) | EN (Enable) |
| :--- | :--- | :--- | :--- | :--- |
| **Motor 1** | Front Left (Roll + Heave) | `GPIO 2` | `GPIO 3` | `GPIO 4` |
| **Motor 2** | Front Right (Roll + Heave) | `GPIO 5` | `GPIO 10` | `GPIO 4` (Shared) |
| **Motor 3** | Rear Center (Pitch + Heave) | `GPIO 20` | `GPIO 21` | `GPIO 4` (Shared) |

> [!NOTE]
> All IBT-2 Enable pins (`R_EN` + `L_EN`) are tied together to **`GPIO 4`** as an active-high hardware Emergency Stop line.

---

### 2. 12-Bit 0–3.3 V Hall Effect Angle Sensors

360° rotatable Hall Position Sensors with 0–3.3 V analog output connected to ESP32 ADC1:

| Joint | Sensor Channel | ESP32-C3 ADC Pin | Resolution |
| :--- | :--- | :--- | :--- |
| **Joint 1** | Motor 1 Position | `GPIO 0` (ADC1_CH0) | $0.088^\circ$ (12-bit) |
| **Joint 2** | Motor 2 Position | `GPIO 1` (ADC1_CH1) | $0.088^\circ$ (12-bit) |
| **Joint 3** | Motor 3 Position | `GPIO 2` (ADC1_CH2)* | $0.088^\circ$ (12-bit) |

*\*Note: If GPIO 2 is used for PWM on C3, ADC1_CH3 on GPIO 3 or external I2C/SPI ADC ADS1115 can be configured in `board_config.h`.*

---

### 3. Display, Status LEDs & Buttons

| Component | Signal | ESP32-C3 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **SSD1306 OLED** | SDA | `GPIO 7` | I2C Data (400 kHz) |
| | SCL | `GPIO 6` | I2C Clock |
| **Status LED** | Built-in | `GPIO 8` | Active LOW |
| **E-Stop Button** | Input | `GPIO 9` | Physical Emergency Stop button (Internal Pullup) |

---

## 📐 0.088° Resolution 12-Bit Hall Angle Sensor Math

The 12-bit ADC converts $0\text{--}3.3\text{ V}$ full-circle analog output to discrete digital counts $D \in [0, 4095]$:

$$\text{Resolution} = \frac{360.0^\circ}{4096} = 0.08789^\circ \approx 0.088^\circ \text{ per ADC count}$$

$$\text{Angle}_\text{raw} = \left( \frac{\text{ADC\_Value}}{4095.0} \right) \times 360.0^\circ$$

$$\text{Angle}_\text{calibrated} = (\text{Angle}_\text{raw} - \text{Zero\_Offset} + 360.0^\circ) \bmod 360.0^\circ$$

---

## 🌐 HTTP REST API Reference

Base URL: `http://motionsimbot1.local:80` (or IP shown on OLED)

| Endpoint | Method | Parameters | Description |
|---|---|---|---|
| `/api/stats` | `GET` | — | Returns JSON with current target vs actual angles, PID error, motor duties, packet rate, RSSI, and E-stop status |
| `/api/pid` | `GET` | `kp`, `ki`, `kd` | Read or update PID loop gains live without rebooting |
| `/api/calibrate` | `POST` | `motor_id` (1-3) | Set current physical position as joint 0° zero-reference point |
| `/api/motor` | `GET` | `id`, `duty` (-1023 to 1023) | Manual motor duty override test (disables UDP mode until reset) |
| `/api/estop` | `POST` | `state` (1 or 0) | Trigger or clear Emergency Stop (disables IBT-2 enable pin) |
| `/api/oled` | `GET` | `mode` (`stats`, `angles`, `pid`, `wifi`) | Switch OLED diagnostic screen |
| `/api/reboot` | `GET` | — | Soft reboot ESP32 |

---

## 🛰️ UDP Motion Control Protocol

- **Port**: `20777` (UDP)
- **Frequency**: `60 Hz` (16.66 ms frame interval)
- **Binary Packet Format** (16 bytes packed):

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;         // 0x4D53494D ("MSIM")
    uint32_t seq;           // Sequence number
    int16_t  target_deg1;   // Motor 1 target angle (deg * 10, e.g. 1800 = 180.0 deg)
    int16_t  target_deg2;   // Motor 2 target angle
    int16_t  target_deg3;   // Motor 3 target angle
    uint16_t checksum;      // XOR checksum
} motion_cmd_t;
```

---

## 🛡️ Closed-Loop PID Tuning & Safety System

1. **Anti-Windup & Output Clamping**:
   - Max Motor Duty is hard-capped at **80% (819 / 1023)** to prevent driver overcurrent.
   - PID integral accumulator is clamped to $\pm 300$ duty.
2. **Directional Dead-Time**:
   - 5 µs dead-time applied during direction flip (forward $\leftrightarrow$ reverse) to prevent shoot-through.
3. **Safety Watchdog**:
   - If no UDP motion command arrives for **>100 ms**, the safety watchdog trips.
   - Motors smoothly ramp duty down to 0 and hold position.
