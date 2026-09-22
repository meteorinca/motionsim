# 📌 MotionSimBot Pinout & Wiring Guide (AS5600 + IBT-2)

This guide details the complete hardware pinout for the **ESP32-S3 MotionSimBot** motion controller, including step-by-step instructions for wiring your **AS5600 magnetic angle encoder directly over I2C** and the **IBT-2 (BTS7960 43A) high-power H-Bridge motor driver**.

> [!NOTE]
> **Active I2C & Motor Pins:**
> - **I2C Bus:** **GPIO 1 (SDA)** and **GPIO 2 (SCL)**.
> - **Motor 1 (Roll):** **GPIO 4 (RPWM)**, **GPIO 5 (LPWM)**, and **GPIO 17 (Shared Enable)**.
> - **Analog Hall sensors (ADC) are completely disabled.** Angle tracking runs 100% digitally through the AS5600 magnetic encoder.

---

## 🏎️ IBT-2 (BTS7960 43A) Motor Driver Wiring Guide

The IBT-2 driver features two connectors: an **8-pin low-voltage logic header** for the ESP32-S3, and **4 heavy-duty screw terminals** for motor power.

```
   ┌──────────────────────────┐                  ┌─────────────────────────────────┐
   │   ESP32-S3 DevKit Board  │                  │      IBT-2 (BTS7960) Driver     │
   │                          │                  │                                 │
   │                      3V3 ├─────────────────►│ VCC (Logic Power)               │
   │                      GND ├─────────────────►│ GND (Common Ground)             │
   │                          │                  │                                 │
   │                  GPIO 17 ├────────┬────────►│ R_EN ──┐ (Jumpered together     │
   │                          │        └────────►│ L_EN ──┘ to GPIO 17)            │
   │                          │                  │                                 │
   │                   GPIO 4 ├─────────────────►│ RPWM (Forward 20 kHz PWM)       │
   │                   GPIO 5 ├─────────────────►│ LPWM (Reverse 20 kHz PWM)       │
   │                          │                  │ R_IS / L_IS (Leave N/C)         │
   └──────────────────────────┘                  └───────────────┬─────────────────┘
                                                                 │ High-Current
                                                                 │ Screw Terminals
       ┌──────────────────────────┐                              │
       │   12V / 24V DC Supply    ├──────► B+ (Positive)         │
       │   (High Current PSU)     ├──────► B- (Ground)           │
       └──────────────────────────┘                              │
                                          M+ (Motor Lead 1) ◄────┤
                                          M- (Motor Lead 2) ◄────┘
```

### IBT-2 8-Pin Logic Header Connection Table

| IBT-2 Pin | ESP32-S3 Pin | Physical Header | Wire Color / Notes |
| :--- | :--- | :--- | :--- |
| **VCC** | **3V3** | Left Header `3V3` | Logic supply (3.3V logic compatible with ESP32-S3). |
| **GND** | **GND** | Right/Left Header | Common logic ground. |
| **R_EN** | **GPIO 17** | Left Header `17` | Right Bridge Enable. **Bridge with jumper to L_EN** so both enable together. |
| **L_EN** | **GPIO 17** | Left Header `17` | Left Bridge Enable. Tied to R_EN and GPIO 17. |
| **RPWM** | **GPIO 4** | Left Header `4` | Forward Direction 20 kHz PWM. |
| **LPWM** | **GPIO 5** | Left Header `5` | Reverse Direction 20 kHz PWM. |
| **R_IS** | *(Leave Disconnected)* | — | Over-current alarm output. Not needed. |
| **L_IS** | *(Leave Disconnected)* | — | Over-current alarm output. Not needed. |

> [!IMPORTANT]
> **Crucial IBT-2 Wiring Rules:**
> 1. **Common Ground:** The ESP32 GND and the 12V/24V Power Supply GND must share a common ground reference.
> 2. **R_EN + L_EN Jumper:** On the IBT-2, bridge `R_EN` and `L_EN` together with a female-to-female jumper wire or solder blob, then connect both to **GPIO 17**. If left floating, the driver will not output any current!
> 3. **Never apply high voltage (12V/24V) to the 8-pin header.** Only wire high current to the `B+` and `B-` screw terminals.

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
    │                   GPIO 1 ├────────────►│ SDA (Pin labeled '1')    │
    │                   GPIO 2 ├────────────►│ SCL (Pin labeled '2')    │
    │                          │             │                          │
    │                      GND ├────────────►│ DIR (Tie to GND for CW)  │
    │                          │             │ GPO / OUT (Leave N/C)    │
    └──────────────────────────┘             └──────────────────────────┘
```

### AS5600 Pin-to-Pin Connection Table

| AS5600 Pin | ESP32-S3 Pin | Physical Header Label | Header Location | Description |
| :--- | :--- | :--- | :--- | :--- |
| **VCC** | **3V3** | `3V3` | Left Header (top) | Power supply (3.3V logic from ESP32 rail). |
| **GND** | **GND** | `GND` | Right/Left Header | Ground reference (common ground). |
| **SDA** | **GPIO 1** | **`1`** | Right Header (Pin 4, below TX/RX) | I2C Serial Data line (cleanly labeled `1`). |
| **SCL** | **GPIO 2** | **`2`** | Right Header (Pin 5, next to `1`) | I2C Serial Clock line (cleanly labeled `2`). |
| **DIR** | **GND** | `GND` | Anywhere on GND rail | Rotation Direction. **Tie to GND** for Clockwise (CW) count increase, or **3V3** for CCW. |
| **GPO / OUT**| *(Unconnected)* | — | — | Analog/PWM output pin. Leave disconnected when reading over digital I2C. |
| **PGO** | *(Unconnected)* | — | — | OTP programming pin. Leave disconnected or tie to GND. |

> [!WARNING]
> **AS5600 3.3V Power Jumper:**
> Most green rectangular/circular AS5600 breakout boards have a small 3-pad solder jumper labeled **3.3V / 5V** on the back:
> - Ensure the solder bridge connects the center pad to the **3.3V** pad.
> - Never feed 5V into the AS5600 if SDA and SCL are directly wired to the ESP32-S3.

> [!TIP]
> **Magnet Placement:**
> - The AS5600 requires a **diametrically magnetized** disc magnet (poles on the flat halves).
> - Position the magnet **0.5 mm to 2.0 mm** centered directly above the AS5600 chip.
> - When aligned properly, the chip sets the `MD` (Magnet Detected) status bit (`STATUS: 0x20`), and the WebUI displays **Magnet OK**.

---

## 🚦 Commissioning Workflow (First Motor Spin)

Once wired:
1. Open the WebUI at: **`http://motionsimbot1.local/`**
2. Notice the system boots in **DISARMED (SAFE)** state.
3. Check the **Magnet Status** indicator — ensure it shows `Magnet OK`.
4. Click **`⚡ ARM MOTORS`**.
5. Under **Manual Motor Commissioning**:
   - Click **`Jog FWD (25%)`**: motor will pulse for 400 ms.
   - Look at the **Actual Angle** in the oscilloscope: did the angle increase or decrease?
   - If the angle decreased when jogging forward, check **`Invert Motor 1 Polarity`**! The firmware saves this setting immediately to flash NVS.
6. Under **Step Angle Response Tests**:
   - Click **`+5°`** or **`-5°`** to test closed-loop position holding!
   - Watch the cyan target line and purple actual line trace smoothly on the live oscilloscope!
7. Click **`⚡ Autocalibrate PD`** to tune stiffness and damping automatically.

---

## 📋 Full ESP32-S3 MotionSimBot Pinout Reference

| Subsystem | Signal / Function | ESP32-S3 GPIO | Physical Header | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Status RGB LED** | Onboard WS2812 NeoPixel | **GPIO 48** | Onboard | Single-wire RMT driver (Green=Running, Amber=Connecting, Red=E-Stop) |
| **I2C Bus (AS5600 & OLED)**| SDA (Data)<br/>SCL (Clock) | **GPIO 1**<br/>**GPIO 2** | Right Header **`1`**<br/>Right Header **`2`** | Standard 100 kHz bus: AS5600 @ 0x36, OLED @ 0x3C |
| **Emergency Stop Button** | E-Stop Input | **GPIO 18** | Left Header `18` | Active LOW, internal pullup & debounce enabled. |
| **Motor 1 (Roll / Front Left)** | RPWM (Forward)<br/>LPWM (Reverse) | **GPIO 4**<br/>**GPIO 5** | Left Header `4`<br/>Left Header `5` | 20 kHz PWM to IBT-2 BTS7960 driver |
| **Motor 2 (Pitch / Front Right)**| RPWM (Forward)<br/>LPWM (Reverse) | **GPIO 6**<br/>**GPIO 7** | Left Header `6`<br/>Left Header `7` | 20 kHz PWM to IBT-2 BTS7960 driver |
| **Motor 3 (Heave / Rear)** | RPWM (Forward)<br/>LPWM (Reverse) | **GPIO 15**<br/>**GPIO 16** | Left Header `15`<br/>Left Header `16` | 20 kHz PWM to IBT-2 BTS7960 driver |
| **Motor Enable Line** | Driver Enable (`R_EN` + `L_EN`) | **GPIO 17** | Left Header `17` | Shared active HIGH enable line across all 3 IBT-2 drivers |
| **Analog Hall Sensors** | *(ADC Channels)* | — | — | **Disabled** (Replaced completely with digital AS5600 encoders) |

---

## 🔌 Later Upgrade: Adding the TCA9548A I2C Multiplexer

When you are ready to add Joint 2 and Joint 3 AS5600 encoders (since all AS5600 chips share the fixed I2C address `0x36`):
1. Connect ESP32-S3 **GPIO 1** (SDA) and **GPIO 2** (SCL) to TCA9548A `SDA` and `SCL`.
2. Connect AS5600 #1 to TCA9548A channel `SD0` / `SC0`.
3. Connect AS5600 #2 to TCA9548A channel `SD1` / `SC1`.
4. Connect AS5600 #3 to TCA9548A channel `SD2` / `SC2`.
5. The firmware auto-detects the multiplexer at address `0x70` on boot and switches automatically to multi-channel mode.
