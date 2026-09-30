# 📌 MotionSimBot Pinout & Wiring Guide (10kΩ Potentiometers + IBT-2)

This guide details the complete hardware pinout for the **ESP32-S3 MotionSimBot** motion controller, including step-by-step instructions for wiring your **10kΩ single-turn potentiometers for analog angle feedback** and the **IBT-2 (BTS7960 43A) high-power H-Bridge motor drivers**.

> [!NOTE]
> **Active Hardware Configuration (ESP32-S3):**
> - **Analog Potentiometer Inputs (ADC1, 12-bit 0–4095):**
>   - **Joint 1 (Roll):** **GPIO 1** (ADC1 Channel 0)
>   - **Joint 2 (Pitch):** **GPIO 2** (ADC1 Channel 1)
>   - **Joint 3 (Heave):** **GPIO 3** (ADC1 Channel 2)
> - **Motor 1 (Roll / Front Left):** **GPIO 4 (RPWM)**, **GPIO 5 (LPWM)**, and **GPIO 17 (Shared Enable)**.
> - **Motor 2 (Pitch / Front Right):** **GPIO 6 (RPWM)**, **GPIO 7 (LPWM)**, and **GPIO 17 (Shared Enable)**.
> - **Motor 3 (Heave / Rear):** **GPIO 15 (RPWM)**, **GPIO 16 (LPWM)**, and **GPIO 17 (Shared Enable)**.
> - **E-Stop Button:** **GPIO 18** (Active LOW, internal pull-up).
> - **Status NeoPixel LED:** **GPIO 48**.
> - **AS5600 I2C magnetic encoders and I2C multiplexers are completely removed.** All position feedback runs directly via analog potentiometers.

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
| **RPWM** | **GPIO 4** (M1)<br/>**GPIO 6** (M2)<br/>**GPIO 15** (M3) | Header pins `4`, `6`, or `15` | Forward Direction 20 kHz PWM. |
| **LPWM** | **GPIO 5** (M1)<br/>**GPIO 7** (M2)<br/>**GPIO 16** (M3) | Header pins `5`, `7`, or `16` | Reverse Direction 20 kHz PWM. |
| **R_IS** | *(Leave Disconnected)* | — | Over-current alarm output. Not needed. |
| **L_IS** | *(Leave Disconnected)* | — | Over-current alarm output. Not needed. |

> [!IMPORTANT]
> **Crucial IBT-2 Wiring Rules:**
> 1. **Common Ground:** The ESP32 GND and the 12V/24V Power Supply GND must share a common ground reference.
> 2. **R_EN + L_EN Jumper:** On each IBT-2, bridge `R_EN` and `L_EN` together with a jumper wire or solder bridge, then connect to **GPIO 17**. If left floating, the driver outputs no current!
> 3. **Multiple Drivers:** All 3 IBT-2 driver enable pins (`R_EN`/`L_EN`) connect in parallel to the single shared enable pin **GPIO 17**.
> 4. **Never apply high voltage (12V/24V) to the 8-pin header.** Only wire high current to the `B+` and `B-` screw terminals.

---

## 🎛️ 10kΩ Potentiometer Angle Sensor Wiring Guide

Each motion joint uses a standard **10kΩ single-turn rotary potentiometer** (linear taper "B10K") directly coupled to the pivot shaft or gearbox output. The wiper outputs a 0–3.3V DC analog voltage proportional to the angle, read by the ESP32-S3's internal 12-bit ADC (0–4095 counts).

```
        ┌──────────────────────────┐             ┌───────────────────────────────────┐
        │   ESP32-S3 DevKit Board  │             │   10kΩ Linear Potentiometer (B10K)│
        │                          │             │                                   │
        │                      3V3 ├────────────►│ Pin 1 (CCW Leg / High Rail 3.3V)  │
        │                          │             │                                   │
        │                   GPIO 1 ├◄────────────┤ Pin 2 (Center Wiper / Analog Out) │
        │                          │             │                                   │
        │                      GND ├────────────►│ Pin 3 (CW Leg / Ground Rail)      │
        └──────────────────────────┘             └───────────────────────────────────┘
```

### Multi-Joint Potentiometer Wiring Diagram

All 3 potentiometers share the common **3.3V** supply rail and **GND** rail. Each wiper routes to its own dedicated ADC pin:

```
 ESP32-S3 3V3 Rail ───┬───────────────────┬───────────────────┐
                      │                   │                   │
                  ┌───┴───┐           ┌───┴───┐           ┌───┴───┐
                  │ Pin 1 │           │ Pin 1 │           │ Pin 1 │
                  │       │           │       │           │       │
                  │ POT 1 │           │ POT 2 │           │ POT 3 │
                  │Joint 1│           │Joint 2│           │Joint 3│
                  │ (Roll)│           │(Pitch)│           │(Heave)│
                  │       │           │       │           │       │
                  │ Pin 2 ├──► GPIO 1 │ Pin 2 ├──► GPIO 2 │ Pin 2 ├──► GPIO 3
                  │       │  (ADC1_0) │       │  (ADC1_1) │       │  (ADC1_2)
                  │ Pin 3 │           │ Pin 3 │           │ Pin 3 │
                  └───┬───┘           └───┬───┘           └───┬───┘
                      │                   │                   │
 ESP32-S3 GND Rail ───┴───────────────────┴───────────────────┘
```

### Potentiometer Pin-to-ESP32-S3 Connection Table

| Joint / Function | Potentiometer Terminal | ESP32-S3 Pin | Physical Header Label | Description |
| :--- | :--- | :--- | :--- | :--- |
| **All Joints** | **Pin 1 (Leg 1)** | **3V3** | `3V3` (Left Header top) | Reference voltage (+3.3V rail). |
| **Joint 1 (Roll)** | **Pin 2 (Wiper)** | **GPIO 1** | **`1`** (Right Header) | ADC1 Channel 0 (0–3.3V analog input). |
| **Joint 2 (Pitch)** | **Pin 2 (Wiper)** | **GPIO 2** | **`2`** (Right Header) | ADC1 Channel 1 (0–3.3V analog input). |
| **Joint 3 (Heave)** | **Pin 2 (Wiper)** | **GPIO 3** | **`3`** (Right Header) | ADC1 Channel 2 (0–3.3V analog input). |
| **All Joints** | **Pin 3 (Leg 2)** | **GND** | `GND` (Any GND pin) | Ground reference (0V rail). |

> [!WARNING]
> **Power Rail Protection:**
> - **ONLY connect the potentiometers to 3.3V (`3V3`).** NEVER connect the potentiometer high-side leg to a 5V or 12V/24V rail. Voltages above 3.3V will permanently destroy the ESP32-S3 ADC input pins!
> - If reversing the pot direction is required (voltage decreases when joint rotates positively), swap Pin 1 and Pin 3 on that potentiometer, or check **Invert Motor Polarity** in the WebUI.

> [!TIP]
> **Noise Reduction for Motor Simulators:**
> - Run potentiometer signal wires away from high-current motor leads (`M+`/`M-`) to avoid electromagnetic interference.
> - Using shielded 3-core cable (with shield grounded at the ESP32 end only) or twisted triplets provides the cleanest analog angle readouts.
> - An optional small 100 nF ceramic capacitor placed between Pin 2 (wiper) and Pin 3 (GND) directly at the board header acts as an analog low-pass filter to eliminate brush noise. The firmware also applies a built-in Exponential Moving Average (EMA) filter.

---

## 🚦 Commissioning Workflow (First Motor Spin)

Once wired:
1. Open the WebUI in your browser: **`http://motionsimbot1.local/`**
2. Notice the system boots in **DISARMED (SAFE)** state.
3. Check the **Potentiometer Readout** in the WebUI:
   - Rotate the joint by hand: verify the **ADC Raw (0–4095)** and **Angle (°)** values change smoothly.
   - Rail fault protection will alert if the reading drops to 0 or saturates at 4095 (indicating a disconnected/shorted wiper).
4. Move the rig to your desired physical home/center position and click **`Set Zero`** (or zero tare) to save the offset to flash NVS.
5. Click **`⚡ ARM MOTORS`**.
6. Under **Manual Motor Commissioning**:
   - Click **`Jog FWD (25%)`**: motor will pulse for 400 ms.
   - Look at the **Actual Angle** in the oscilloscope: did the angle change in the expected direction?
   - If the angle moved opposite to the target, click **`Invert Motor 1 Polarity`**! The firmware saves this setting immediately to flash NVS.
7. Under **Step Angle Response Tests**:
   - Click **`+5°`** or **`-5°`** to test closed-loop position holding!
   - Watch the cyan target line and purple actual line trace smoothly on the live oscilloscope.
8. Click **`⚡ Autocalibrate PD`** to tune stiffness and damping automatically.

---

## 📋 Full ESP32-S3 MotionSimBot Pinout Reference

| Subsystem | Signal / Function | ESP32-S3 GPIO | ADC Channel | Physical Header | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Status RGB LED** | Onboard WS2812 NeoPixel | **GPIO 48** | — | Onboard | Single-wire RMT driver (Green=Running, Amber=Connecting, Red=E-Stop) |
| **Emergency Stop Button** | E-Stop Input | **GPIO 18** | — | Left Header `18` | Active LOW, internal pull-up & debounce enabled |
| **Potentiometer Joint 1** | Joint 1 Wiper (Roll) | **GPIO 1** | ADC1 Channel 0 | Right Header `1` | 10kΩ linear pot, 0–3.3V analog input |
| **Potentiometer Joint 2** | Joint 2 Wiper (Pitch) | **GPIO 2** | ADC1 Channel 1 | Right Header `2` | 10kΩ linear pot, 0–3.3V analog input |
| **Potentiometer Joint 3** | Joint 3 Wiper (Heave) | **GPIO 3** | ADC1 Channel 2 | Right Header `3` | 10kΩ linear pot, 0–3.3V analog input |
| **Motor 1 (Roll / Front Left)** | RPWM (Forward)<br/>LPWM (Reverse) | **GPIO 4**<br/>**GPIO 5** | — | Left Header `4`<br/>Left Header `5` | 20 kHz PWM to IBT-2 BTS7960 driver |
| **Motor 2 (Pitch / Front Right)**| RPWM (Forward)<br/>LPWM (Reverse) | **GPIO 6**<br/>**GPIO 7** | — | Left Header `6`<br/>Left Header `7` | 20 kHz PWM to IBT-2 BTS7960 driver |
| **Motor 3 (Heave / Rear)** | RPWM (Forward)<br/>LPWM (Reverse) | **GPIO 15**<br/>**GPIO 16** | — | Left Header `15`<br/>Left Header `16` | 20 kHz PWM to IBT-2 BTS7960 driver |
| **Motor Enable Line** | Driver Enable (`R_EN` + `L_EN`) | **GPIO 17** | — | Left Header `17` | Shared active HIGH enable line across all 3 IBT-2 drivers |
| **OLED Display (Optional)** | SDA (Data)<br/>SCL (Clock) | **GPIO 8**<br/>**GPIO 9** | — | Header pins `8`, `9` | I2C display bus (SSD1306 @ 0x3C, reserved) |

---

## 📌 ESP32-C3 MotionSimBot Pinout Reference (Alternative Board)

If compiling for the compact **ESP32-C3** target (`boards/esp32c3_motionsimbot`):

| Subsystem | Signal / Function | ESP32-C3 GPIO | ADC Channel | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Status LED** | Standard LED | **GPIO 8** | — | Active LOW |
| **E-Stop Button** | Emergency Stop | **GPIO 9** | — | Active LOW with pull-up |
| **Potentiometer Joint 1** | Joint 1 Wiper (Roll) | **GPIO 0** | ADC1 Channel 0 | 10kΩ pot wiper |
| **Potentiometer Joint 2** | Joint 2 Wiper (Pitch) | **GPIO 1** | ADC1 Channel 1 | 10kΩ pot wiper |
| **Potentiometer Joint 3** | Joint 3 Wiper (Heave) | **GPIO 2** | ADC1 Channel 2 | 10kΩ pot wiper |
| **Motor 1** | RPWM / LPWM | **GPIO 2** / **GPIO 3** | — | 20 kHz PWM |
| **Motor 2** | RPWM / LPWM | **GPIO 5** / **GPIO 10** | — | 20 kHz PWM |
| **Motor 3** | RPWM / LPWM | **GPIO 20** / **GPIO 21** | — | 20 kHz PWM |
| **Motor Enable** | Shared `R_EN` + `L_EN` | **GPIO 4** | — | Shared enable line |
| **OLED Display** | SDA / SCL | **GPIO 7** / **GPIO 6** | — | I2C display |
