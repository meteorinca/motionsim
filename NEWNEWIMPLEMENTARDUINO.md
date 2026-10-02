# NEWNEWIMPLEMENTARDUINO — Switch to Arduino UNO Motor Controller

## Why

The ESP32-S3 ADC is noisy (~26 counts peak-to-peak = ~2.3°) due to its SAR
architecture, shared reference with the RF subsystem, and motor PWM coupling.
The Arduino UNO's AVR ADC tested at **±1 count** on the same pot with the same
wiring — a 13× improvement before any filtering.

SMC3 (Simulator Motor Controller for 3 Motors) is the proven open-source
firmware for exactly this use case, running on the UNO R3. We build a
**minimalist custom variant** — same idea, our own protocol.

---

## Control Strategy: Simple PD, Not PID

**The PC does the hard work. The Arduino just follows.**

Our custom SimTools alternative already sends **smooth, pre-ramped setpoints at
100 Hz**. The Arduino never sees a step input — every target it receives is
already motion-profiled on the PC side. This changes everything about what the
controller needs to do:

```
PC (100 Hz)                         Arduino (4096 Hz)
-----------                         -----------------
Motion profile + ramp   -->  serial -->  error = target - actual
Soft limits                              output = Kp*error - Kd*velocity
Coordinate transforms                    PWM -> motor
```

The control loop is therefore:

```
output = Kp * error - Kd * velocity
```

- **Kp** — stiffness: how hard the motor chases the target
- **Kd** — damping: brakes against velocity to prevent overshoot
- **No Ki** — no steady-state offset to eliminate because the PC's pre-ramped
  target arrives right on top of the motor position. If there's a residual
  offset, raise Kp slightly.
- **No trajectory profiler** — the PC already did this
- **No feedforward** — not needed when the setpoint is already smooth
- **No stiction model, no β, no k_aw** — those are compensation for step
  inputs; irrelevant here

**Tuning ladder:**
1. Ki=0, Kd=0 — raise Kp until it tracks well but just starts to oscillate
2. Back Kp off 20%, then add Kd until oscillation damps cleanly
3. Done. That's the whole tune.

---

## What We Keep vs. Drop

| Feature | ESP32 version | Arduino version |
|---|---|---|
| Control loop | complex PID + trajectory + feedforward | simple PD |
| Potentiometer ADC (10-bit) | noisy (~2.3°) | clean (±0.35°) |
| Motor PWM + direction | LEDC peripheral | Timer1/2 hardware PWM |
| E-stop / soft limits | on-chip | on-chip (simple) |
| Per-joint zero calibration | NVS (flash) | EEPROM |
| Serial control protocol | UDP over WiFi | USB-Serial 500 kbaud |
| Motion profiling / ramping | on-chip (untested) | PC side (proven) |
| Web dashboard / tuning UI | yes | replaced by PC app |
| WiFi / mDNS / OTA | yes | dropped |
| OLED display | yes | dropped (LED only) |

---

## Hardware Map — Arduino UNO R3

```
Analog inputs (ADC, free-running ISR)
  A0 -> Joint 1 pot wiper
  A1 -> Joint 2 pot wiper
  A2 -> Joint 3 pot wiper

PWM outputs (Timer1 = 31.25 kHz, Timer2 = 31.25 kHz phase-correct)
  D9  -> Motor 1 PWM    (OC1A)
  D10 -> Motor 2 PWM    (OC1B)
  D11 -> Motor 3 PWM    (OC2A)

Direction pins (digital)
  D4  -> Motor 1 DIR
  D5  -> Motor 2 DIR
  D7  -> Motor 3 DIR

E-stop input
  D2  -> Hardware e-stop (INT0, pulled up)

Status LED
  D13 -> Built-in LED (armed / fault blink codes)

UART
  USB (D0/D1) -> PC at 500,000 baud
```

> **Motor driver:** IBT-2 (43 A, RPWM/LPWM) or MonsterMoto shield.
> Wiring matches SMC3 MODE 1 (PWM + DIR) to match existing H-bridge wiring.

---

## Firmware Architecture

```
Timer2 OVF ISR (~4096 Hz)   <- heartbeat tick
|
+-- Read ADC (free-running, ISR-fed ring buffer)
|   median-of-9 -> EMA -> angle (10-bit, 0-1023)
|
+-- PD loop (all 3 joints)
|   error    = target - actual
|   velocity = (actual - last_actual) / dt   [filtered]
|   output   = Kp*error - Kd*velocity
|   clamp to 80% duty, apply deadband, e-stop check
|
+-- Write PWM duty to Timer1/Timer2

UART RX (interrupt-driven, 500 kbaud)
  <- [0xFF] T1_hi T1_lo T2_hi T2_lo T3_hi T3_lo   (100 Hz from PC)
  -> [0xFF] A1_hi A1_lo A2_hi A2_lo A3_hi A3_lo   (actual positions)

EEPROM
  <- Kp, Kd, deadband, zero offsets on startup
  -> save on command byte
```

### Serial Protocol (SMC3-compatible subset)

```
PC -> Arduino  (7 bytes, 100 Hz)
  [0xFF] [T1_hi] [T1_lo] [T2_hi] [T2_lo] [T3_hi] [T3_lo]
  Target = 10-bit value (0-1023)

Arduino -> PC  (7 bytes per reply)
  [0xFF] [A1_hi] [A1_lo] [A2_hi] [A2_lo] [A3_hi] [A3_lo]
  Actual = 10-bit ADC reading
```

---

## Implementation Steps

### Step 1 — Hardware test (done)
- [x] Verify pot noise on Arduino (±1 count — clean)
- [ ] Wire IBT-2 / MonsterMoto to UNO pins above
- [ ] Confirm motor spins both directions with manual PWM sketch

### Step 2 — Core firmware (boards/arduino/arduinomosim/)
1. `adc.cpp`         — free-running ADC ISR, ring buffer, median-9 + EMA
2. `motor.cpp`       — Timer1/2 phase-correct PWM, set(id, duty),
                       direction inversion, 80% clamp, e-stop cut
3. `pd_ctrl.cpp`     — `output = Kp*error - Kd*velocity`, deadband,
                       velocity filter (EMA alpha=0.25), duty clamp
4. `serial_ctrl.cpp` — SMC3-compatible 7-byte rx/tx, zero/param commands
5. `eeprom_cfg.cpp`  — load/save Kp, Kd, deadband, zero offsets
6. `arduinomosim.ino`— Timer2 OVF ISR at 4 kHz wires it all together

### Step 3 — PC dashboard bridge
- Swap UDP socket in bench_pc_dashboard.py for serial.Serial at 500 kbaud.
  Same data model, same plot UI. PC side continues to own motion profiling.

### Step 4 — Calibration & first tune
- Zero-offset: send `0xFE joint_id` -> Arduino saves current ADC to EEPROM
- Param update: send `0xFD param_id val_hi val_lo` (Kp, Kd, deadband)
- Start with Kp=5, Kd=1, no deadband. Raise Kp until oscillation, back off
  20%, add Kd until clean. Takes ~10 minutes on a real rig.

---

## Key AVR Design Decisions

| Decision | Choice | Reason |
|---|---|---|
| ADC prescaler | 128 (125 kHz clock) | Best SNR on AVR |
| Control tick rate | 4096 Hz (Timer2 OVF) | Matches SMC3, deterministic |
| PWM frequency | 31.25 kHz phase-correct | Above hearing; no coil whine |
| Baud rate | 500,000 | SMC3 standard; near-zero CPU overhead |
| EEPROM writes | On command only | 100k cycle limit on AVR |
| Pot filter | Median-9 -> EMA alpha=0.1 | Proven clean from noise test |
| Velocity filter | EMA alpha=0.25 on raw delta | Smooth D-term, no spike amplification |

---

## What We Lose (and the Mitigation)

- **No web UI** — tune via serial commands from bench_pc_dashboard.py. Only
  two knobs (Kp, Kd) to adjust anyway.
- **No OTA** — flash via USB. Fine for a bench controller.
- **No WiFi targets** — PC sends targets over serial. The racesync/ UDP
  pipeline still runs entirely on the PC side.
- **10-bit vs 12-bit** — 0.35°/count vs 0.088°/count. For ±45° range that's
  257 steps; with 13× lower noise the effective resolution is better overall.
- **No integral** — if there's a persistent positional offset under load, a
  small Kp bump fixes it. Rig weight/gravity loads are constant and handled by
  the PC's output scaling, not firmware integral windup.
