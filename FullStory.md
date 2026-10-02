#  Complete Architecture & Component Guide: MotionSim (SMC3), SimSync & RaceSync

This document outlines the simplified, rock-solid hardware and software ecosystem for the 3DOF motion simulator rig, now standardized on the **Arduino UNO R3 running SMC3 (Mode 2)**.

---

## 1. Updated Architecture Overview

```mermaid
flowchart TD
    GAME[" DiRT Rally 2.0 / Racing Sim<br/>(Outputs UDP Telemetry on Port 20777)"]

    subgraph PC_LAYER [" PC Software Layer"]
        RACESYNC[" RaceSync (Web Dashboard)<br/>• Big-screen track progress & GPS map<br/>• Pseudo-multiplayer / spectator view<br/>• Live telemetry recording & replay"]
        SIMSYNC[" SimSync (Go Motion Engine) / SMC3Utils<br/>• Replaces SimTools (zero bloat, single binary)<br/>• 3DOF kinematics: G-forces ➔ Pitch/Roll/Heave<br/>• High-speed USB Serial Bridge (500k baud)<br/>• Formats setpoints [Axx][Bxx][Cxx] to Arduino"]
    end

    subgraph HW_CONTROLLER [" Hardware Controller (Arduino UNO R3)"]
        SMC3[" Arduino UNO R3 (SMC3 Firmware - Mode 2)<br/>• 4 kHz instantaneous PID loop<br/>• Clean ±1 count 10-bit AVR ADC sampling<br/>• Direct hardware PWM & direction control<br/>• Battle-tested reverse braking & clipping safety"]
    end

    subgraph RIG [" Physical Motion Rig Hardware"]
        DRIVERS[" 3x IBT-2 Motor Drivers (BTS7960 43A)"]
        MOTORS[" 3x 24V DC Actuator Motors (Roll, Pitch, Heave)"]
        POTS[" 3x 10k Feedback Potentiometers (0-5V)"]
    end

    %% Telemetry Stream
    GAME -->|UDP 20777 Broadcast| RACESYNC
    GAME -->|UDP 20777 Broadcast| SIMSYNC

    %% PC to Hardware
    SIMSYNC -->|USB Serial @ 500000 baud| SMC3

    %% Hardware Control Loop
    POTS -->|Analog 0-5V Feedback (A0, A1, A2)| SMC3
    SMC3 -->|PWM + DIR + EN (Pins 2-11)| DRIVERS
    DRIVERS -->|High Current 24V Drive| MOTORS
    MOTORS -.->|Physical Coupling| POTS
```

---

## 2. Why We Simplified to Arduino UNO (SMC3)

1. **Eliminated ADC Noise:** The ESP32-S3 internal SAR ADC had ~26 counts of peak-to-peak jitter due to RF/PWM switching noise. The Arduino UNO's AVR ADC delivers **±1 count precision**, completely eliminating motor buzzing and position chatter.
2. **Zero Inter-Chip Lag:** Both the ADC position reading and the PWM motor control happen inside the same ATmega328P chip at **4,000 Hz**. No UART bridges, level-shifter resistors, or inter-MCU framing latency.
3. **Proven Reliability:** SMC3 is the motion sim community's gold standard firmware for 3DOF rigs. It includes built-in reverse current braking, software travel limits, stall protection, and deadzone filtering out-of-the-box.
4. **Retired ESP32-S3 code:** `motionsimbot` and `motionsimbench` have been completely removed from the workspace in favor of [`SMC3/SMC3.ino`](file:///c:/Users/dontm/Documents/mojCodexstuff/ActiveGithub/motionsim/SMC3/SMC3.ino).

---

## 3. Complete Pinout & Wiring: Arduino UNO to 3x IBT-2 Drivers

In [`SMC3/SMC3.ino`](file:///c:/Users/dontm/Documents/mojCodexstuff/ActiveGithub/motionsim/SMC3/SMC3.ino), **`MODE2`** is enabled for the BTS7960 / IBT-2 43A H-Bridges.

### Wiring Table

| Signal | Motor 1 (Roll / Front L) | Motor 2 (Pitch / Front R) | Motor 3 (Heave / Rear) | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **RPWM (Speed / PWM)** | **Pin 9** | **Pin 10** | **Pin 11** | High-speed hardware PWM |
| **LPWM (Direction)** | **Pin 2** | **Pin 4** | **Pin 6** | Digital Direction output |
| **R_EN & L_EN (Driver Enable)** | **Pin 3** | **Pin 5** | **Pin 7** | Tie IBT-2 `R_EN` and `L_EN` together |
| **Feedback Pot (Wiper)** | **A0** | **A1** | **A2** | Connect to center pin of 10k pot |
| **Motion Scaler (Optional)** | — | — | **A5** | Optional global scale potentiometer |

### Logic Power & Common Grounds
* **IBT-2 `VCC`:** Connect all 3 IBT-2 `VCC` pins to Arduino **5V**.
* **IBT-2 `GND`:** Connect all 3 IBT-2 `GND` pins to Arduino **GND**.
* **Potentiometer Rails:** Connect pot outer pins to Arduino **5V** and **GND**.
* **Motor Power Supply:** Ensure 24V Power Supply **Negative (- / 0V)** connects to Arduino **GND** for a clean common ground.

---

## 4. Component Roles in the Full Stack

| Component | Location | Role |
| :--- | :--- | :--- |
| **SMC3 Firmware** | [`SMC3/SMC3.ino`](file:///c:/Users/dontm/Documents/mojCodexstuff/ActiveGithub/motionsim/SMC3/SMC3.ino) | Runs on Arduino UNO. Manages 4 kHz closed-loop PID, reads pots (A0-A2), drives 3x IBT-2s in Mode 2. Listens at 500,000 baud. |
| **SimSync** | `../simsync` | Go-native PC bridge. Reads game UDP 20777, calculates pitch/roll/heave washouts, and streams `[Axx][Bxx][Cxx]` commands over USB serial. |
| **RaceSync** | `../racesync` | Dedicated spectator / side-monitor visualizer for DiRT Rally 2.0 (track map, stage progress, and live UDP session recording/playback). |

---

## 5. Next Steps

1. **Flash Arduino UNO:** Open [`SMC3/SMC3.ino`](file:///c:/Users/dontm/Documents/mojCodexstuff/ActiveGithub/motionsim/SMC3/SMC3.ino) in Arduino IDE and upload to your Uno board (Board: *Arduino Uno*, Port: *COMx*).
2. **First Power-Up Check:** Use `SMC3Utils` on Windows or `simsync` in bench mode to verify pot tracking before engaging 24V motor power.