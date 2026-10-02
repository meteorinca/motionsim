# 🏎️ Complete Architecture & Component Guide: MotionSim, SimSync & RaceSync

This document outlines the complete hardware, firmware, and software ecosystem powering the 3DOF motion simulator rig, the telemetry pipeline, and clarifies the relationship between the active repositories.

---

## 🏗️ 1. Executive Architecture Overview

```mermaid
flowchart TD
    GAME["🎮 DiRT Rally 2.0 / Racing Sim<br/>(Outputs UDP Telemetry on Port 20777)"]

    subgraph PC_LAYER ["🖥️ PC Software Layer"]
        RACESYNC["🏁 RaceSync (Web Dashboard)<br/>• Big-screen track progress & GPS map<br/>• Pseudo-multiplayer / spectator view<br/>• Live telemetry recording & replay"]
        SIMSYNC["⚡ SimSync (Go Motion Engine)<br/>• Replaces SimTools (zero bloat, single binary)<br/>• 3DOF kinematics: G-forces ➔ Pitch/Roll/Heave<br/>• High-speed USB Serial Bridge<br/>• Web Oscilloscope & live PID tuner (localhost:7070)"]
    end

    subgraph HW_CONTROLLERS ["🔌 Dual-MCU Hardware Controller Layer"]
        ARDUINO["📟 Arduino UNO (Dedicated ADC Sensor Node)<br/>• Clean ±1 count 10-bit AVR ADC sampling<br/>• Reads feedback potentiometers (Joints 1, 2, 3)"]
        SHIFTER["⚡ Level Shifter / Voltage Divider<br/>Arduino TX (5V) ➔ 3.3V ➔ ESP32-S3 RX<br/>Shared Common GND"]
        ESP["🧠 ESP32-S3 Motion Controller<br/>• High-speed 100 Hz closed-loop motor control<br/>• Receives setpoints from SimSync via USB-C<br/>• Receives clean angles from Arduino via UART<br/>• Emergency Stop supervisor & WebUI"]
    end

    subgraph RIG ["⚙️ Physical Motion Rig Hardware"]
        DRIVERS["⚡ 3x IBT-2 Motor Drivers (BTS7960 43A)"]
        MOTORS["🏎️ 3x 24V DC Actuator Motors (Roll, Pitch, Heave)"]
        POTS["📐 Physical Feedback Potentiometers"]
    end

    %% Telemetry Stream
    GAME -->|UDP 20777 Broadcast| RACESYNC
    GAME -->|UDP 20777 Broadcast| SIMSYNC

    %% PC to Hardware
    SIMSYNC -->|USB-C Serial @ 921600 baud| ESP

    %% Dual MCU Signal Flow
    POTS -->|Low-noise Analog 0-5V| ARDUINO
    ARDUINO -->|UART Stream (5V TX)| SHIFTER
    SHIFTER -->|UART Stream (3.3V RX)| ESP

    %% Actuation Loop
    ESP -->|20 kHz PWM & EN Signals| DRIVERS
    DRIVERS -->|High Current 24V Drive| MOTORS
    MOTORS -.->|Physical Coupling| POTS
    ESP -.->|Live telemetry & joint feedback| SIMSYNC
```

---

## 🧩 2. What Each Component Does

### A. `motionsim` (This Workspace)
* **What it is:** The central hardware firmware, rig electronics pinouts, and commissioning codebase.
* **Firmware components:**
  * `motionsimbot/`: ESP-IDF v5.5 C firmware running on the ESP32-S3. Handles the motor PWM generation (IBT-2 H-bridges), safety supervisor (hardware/software E-Stop), emergency timeouts, and serial communication.
  * `motionsimbench/`: Bench testing and calibration firmware.
  * `buildbinesp32s3.ps1` & `flashbinesp32s3.ps1`: Automated build and flashing scripts.

### B. `racesync` (Race Telemetry & Control Room)
* **What it is:** A web-based telemetry visualizer and control room frontend.
* **Primary Use Case:**
  * **Big Screen Visualizer:** Displayed on a dedicated side monitor or television while driving.
  * **Track Progress & GPS:** Real-time 2D track map rendering vehicle position on rally stages.
  * **Pseudo-Multiplayer:** Compare your stage splits and ghost trajectories against recorded runs.
  * **Telemetry Recording & Replay:** Captures raw UDP sessions to analyze driver inputs (throttle, braking, steering angle, G-forces) after the race.

### C. `simsync` (`../simsync`)
* **What it is:** A standalone, high-performance Go-native motion simulator control suite (`simsync.exe`) — our lean, purpose-built alternative to commercial software like SimTools or SimFeedback.
* **Core Responsibilities:**
  1. **UDP Ingestion:** Listens to game physics packets on UDP port 20777.
  2. **Kinematic Transformation:** Converts raw surge, sway, heave G-forces and suspension displacements into target actuator angles for Roll, Pitch, and Heave.
  3. **USB-C Serial Bridge:** Pumps smooth pre-ramped targets to the motion controller at 100 Hz.
  4. **Oscilloscope & Live PID Tuner:** Serves `http://localhost:7070` with a 60 FPS real-time oscilloscope (Target vs. Actual vs. Error vs. PWM Duty) and sliders to tune PID gains on the fly.

---

## ⚡ 3. Dual-MCU Architecture: Why Arduino + ESP32-S3?

### The Problem: ESP32-S3 ADC Noise
The ESP32-S3 internal ADC uses a Successive Approximation Register (SAR) architecture that shares internal references with the high-frequency 2.4 GHz RF (Wi-Fi/Bluetooth) circuitry. When driving high-current inductive motor loads over PWM (IBT-2 H-bridges switching 24V), the ESP32 ADC exhibits severe noise:
* Peak-to-peak jitter of **~26 ADC counts** (~2.3° of angular uncertainty).
* This noise causes motor hunting, audible hum, and unnecessary driver heat.

### The Solution: Arduino UNO as a Dedicated ADC Front-End
The Arduino UNO’s ATmega328P has a rock-solid, isolated 10-bit AVR ADC:
* Measured jitter is only **±1 ADC count** on the identical potentiometer and wiring (a **13× improvement**).

### Inter-Board Communication (UART Bridge)
Rather than abandoning the ESP32-S3's superior processing power, high PWM resolution, and networking, we adopt a hybrid architecture:

```
[ Feedback Potentiometers (0-5V) ]
                │
                ▼
   [ Arduino UNO (Analog A0, A1, A2) ]
                │
         (TX Pin 1 @ 5V)
                │
                ▼
   ┌───────────────────────────┐
   │ Voltage Divider / Shifter │
   │ 1kΩ (TX ➔ ESP RX)         │
   │ 2kΩ (ESP RX ➔ GND)        │
   │ (Steps 5.0V down to 3.3V) │
   └───────────────────────────┘
                │
                ▼
   [ ESP32-S3 (RX Pin @ 3.3V) ]
                │
         (Shared GND Bus)
```

1. **Sampling:** Arduino UNO continuously reads the 3 feedback pots with hardware oversampling.
2. **Transmission:** Sends a compact binary/ASCII frame over hardware Serial (e.g. `115200` or `250000` baud) with checksum:
   $$\text{Frame: } \langle\text{SYNC}, \theta_1, \theta_2, \theta_3, \text{CRC}\rangle$$
3. **Level Shifting:** Arduino operates at 5V logic; ESP32-S3 operates at 3.3V logic (non-5V tolerant).
   * **Arduino TX (5V) ➔ 1kΩ / 2kΩ resistor divider (or bi-directional level shifter) ➔ ESP32-S3 RX (3.3V)**.
   * **Crucial:** Arduino GND and ESP32-S3 GND must be tied together.
4. **Execution:** The ESP32-S3 receives the clean joint angles, runs the motor control loop, and issues 20 kHz PWM signals to the IBT-2 drivers.

---

## 🔍 4. Verification: The Two `racesync` Folders

### What is the Difference?
Both directories belong to `https://github.com/meteorinca/racesync.git`, but they are on different branches:

| Location | Branch | Latest Commit | Key Differences |
| :--- | :--- | :--- | :--- |
| **`ActiveGithub/racesync`** (`../racesync`) | `geminiv2` | `bd022c9` | Standard control room UI with modular files (`css/style.css`, `js/app.js`, `public/stage_data/`). |
| **`motionsim/racesync`** (Inner Folder) | `geminav1` | `7e8d16e` | Contains `d577ada` and `7e8d16e`, which added the **live UDP telemetry recording and replay engine** (`js/app.js` expanded with playback scrubber, session recording, and telemetry export). |

### Conclusion & Cleanup Recommendation
You noted that `ActiveGithub/racesync` is your intended canonical project folder and `motionsim/racesync` was a temporary local copy-paste.

To consolidate everything cleanly:
1. **Merge / fast-forward `ActiveGithub/racesync`:** Check out `geminav1` (or merge `geminav1` into `geminiv2`) in `../racesync` so you retain the telemetry recording and replay features:
   ```powershell
   cd ..\racesync
   git checkout geminav1  # or git merge origin/geminav1
   ```
2. **Remove the nested copy:** Once you verify `ActiveGithub/racesync` has the features you want, you can safely delete `motionsim/racesync` to avoid nested git repository warnings and prevent confusion:
   ```powershell
   Remove-Item -Recurse -Force .\racesync
   ```