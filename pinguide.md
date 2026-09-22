# 📌 MotionSimBot Pinout & AS5600 Wiring Guide

This guide details the complete hardware pinout for the **ESP32-S3 MotionSimBot** motion controller and step-by-step instructions for wiring your **first AS5600 magnetic angle encoder directly over I2C (without a multiplexer)**.

---

## 🧲 Direct Wiring: First AS5600 (No Multiplexer)

The firmware automatically senses whether a TCA9548A I2C multiplexer is attached. If absent, it runs in **Direct I2C Mode**, reading the primary AS5600 encoder on Channel 0 at I2C address `0x36`.

```
    ┌──────────────────────────┐             ┌──────────────────────────┐
    │   ESP32-S3 DevKit Board  │             │   AS5600 Encoder Board   │
    │                          │             │                          │
    │                      3V3 ├────────────►│ VCC (3.3V)               │
    │                      GND ├────────────►│ GND                      │
    │                          │             │                          │
    │                   GPIO 8 ├────────────►│ SDA                      │
    │                   GPIO 9 ├────────────►│ SCL                      │
    │                          │             │                          │
    │                      GND ├────────────►│ DIR (Tie to GND for CW)  │
    │                          │             │ GPO / OUT (Leave N/C)    │
    └──────────────────────────┘             └──────────────────────────┘
```

### Pin-to-Pin Connection Table

| AS5600 Pin | ESP32-S3 Pin | Function / Description |
| :--- | :--- | :--- |
| **VCC** | **3V3** (3.3V) | Power supply (3.3V logic). *Do not connect to 5V without checking board solder bridge!* |
| **GND** | **GND** | Ground reference (common ground). |
| **SDA** | **GPIO 8** | I2C Serial Data line. |
| **SCL** | **GPIO 9** | I2C Serial Clock line (configured at 400 kHz). |
| **DIR** | **GND** | Rotation Direction. **Tie to GND** for Clockwise (CW) count increase, or **3V3** for CCW. |
| **GPO / OUT**| *(Unconnected)* | Analog/PWM output pin. Leave disconnected when reading over digital I2C. |
| **PGO** | *(Unconnected)* | OTP programming pin. Leave disconnected or tie to GND. |

> [!IMPORTANT]
> **AS5600 3.3V Power Jumper:**
> Most green rectangular/circular AS5600 breakout boards have a small 3-pad solder jumper labeled **3.3V / 5V** on the back or side:
> - When powering with **3.3V** from the ESP32-S3 3V3 rail, ensure the solder bridge connects the center pad to the **3.3V** pad (or VCC directly bypasses the on-chip 5V-to-3.3V LDO).
> - Never feed 5V into the AS5600 if SDA and SCL are directly wired to the ESP32-S3, as the I2C pullups will pull the data lines to 5V and risk damaging the ESP32-S3 GPIOs.

> [!TIP]
> **Magnet Placement (Crucial for AS5600):**
> - The AS5600 requires a **diametrically magnetized** disc magnet (poles on the flat halves, not the top/bottom faces).
> - Position the magnet **0.5 mm to 2.0 mm** centered directly above the AS5600 chip.
> - When aligned properly, the chip sets the `MD` (Magnet Detected) status bit (`STATUS: 0x20`).

---

## 🚦 Live Telemetry & Verification

Once wired:
1. Boot the ESP32-S3 and connect to your WiFi or monitor serial on `COM9`:
   ```text
   I (442) AS5600: Direct I2C connection (no TCA9548A multiplexer detected)
   I (452) AS5600: AS5600 Encoder detected on channel 0 (STATUS: 0x20, MD: 1, ML: 0, MH: 0)
   ```
2. Open your web browser and go to:
   👉 **`http://motionsimbot1.local/`** (or the ESP32's assigned IP address).
3. The dashboard will display live degrees ($0.0^\circ$ to $360.0^\circ$) in real time as you rotate the motor shaft or magnet.

---

## 📋 Full ESP32-S3 MotionSimBot Pinout Reference

| Subsystem | Signal / Function | ESP32-S3 GPIO | Hardware Notes |
| :--- | :--- | :--- | :--- |
| **Status RGB LED** | Onboard WS2812 NeoPixel | **GPIO 48** | Built-in single-wire RMT driver (Green=Running, Amber=Connecting, Red=E-Stop) |
| **I2C Bus (Sensors/Display)**| SDA (Data)<br/>SCL (Clock) | **GPIO 8**<br/>**GPIO 9** | Shared 400 kHz bus: AS5600 encoder (0x36) & SSD1306 OLED (0x3C) |
| **Emergency Stop Button** | E-Stop Input | **GPIO 18** | Active LOW, internal pullup enabled. Momentary or latching switch to GND. |
| **Motor 1 (Roll / Front Left)** | RPWM (Forward)<br/>LPWM (Reverse) | **GPIO 4**<br/>**GPIO 5** | 20 kHz PWM to IBT-2 BTS7960 driver |
| **Motor 2 (Pitch / Front Right)**| RPWM (Forward)<br/>LPWM (Reverse) | **GPIO 6**<br/>**GPIO 7** | 20 kHz PWM to IBT-2 BTS7960 driver |
| **Motor 3 (Heave / Rear)** | RPWM (Forward)<br/>LPWM (Reverse) | **GPIO 15**<br/>**GPIO 16** | 20 kHz PWM to IBT-2 BTS7960 driver |
| **Motor Enable Line** | Driver Enable (`R_EN` + `L_EN`) | **GPIO 17** | Shared active HIGH enable line across all 3 IBT-2 drivers |
| **Analog Hall Sensor 1** | ADC1_CH0 | **GPIO 1** | Joint 1 12-bit analog fallback ($0.088^\circ$/count) |
| **Analog Hall Sensor 2** | ADC1_CH1 | **GPIO 2** | Joint 2 12-bit analog feedback |
| **Analog Hall Sensor 3** | ADC1_CH2 | **GPIO 3** | Joint 3 12-bit analog feedback |

---

## 🔌 Later Upgrade: Adding the TCA9548A I2C Multiplexer

When you are ready to add Joint 2 and Joint 3 AS5600 encoders (since all AS5600 chips share the fixed I2C address `0x36`):
1. Connect ESP32-S3 `GPIO 8` (SDA) and `GPIO 9` (SCL) to TCA9548A `SDA` and `SCL`.
2. Connect AS5600 #1 to TCA9548A channel `SD0` / `SC0`.
3. Connect AS5600 #2 to TCA9548A channel `SD1` / `SC1`.
4. Connect AS5600 #3 to TCA9548A channel `SD2` / `SC2`.
5. The firmware will auto-detect the multiplexer at address `0x70` on boot and switch automatically to multi-channel mode.
