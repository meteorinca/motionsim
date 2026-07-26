
## Phase 0: Infrastructure & Bench Validation (Week 1)

### Router Configuration
| Setting | Value | Why |
|---------|-------|-----|
| **5 GHz SSID** | `MOTION_SIM_5G` | Dedicated, no other devices |
| **2.4 GHz SSID** | `MOTION_SIM_2G4` | Fallback / management only |
| **5 GHz channel** | 36, 40, 44, or 149 (UNII-1 or UNII-3) | Cleanest, least radar interference |
| **Channel width** | 40 MHz | Balance of speed vs stability |
| **Security** | WPA3 or WPA2-AES | WPA2-TKIP adds latency |
| **DHCP lease time** | 86400s (24h) or static IPs | Avoid mid-race DHCP churn |
| **QoS / WMM** | Enabled | Prioritizes UDP gaming traffic |
| **Multicast rate** | Set to lowest basic rate (6 Mbps) | Ensures telemetry broadcasts reliably |

**Static IP assignments:**
| Device | IP | Notes |
|--------|-----|-------|
| Router | 192.168.4.1 | Gateway |
| PC1 (Rig A) | 192.168.4.10 | Bridge + game |
| ESP32-C6 Rig A | 192.168.4.110 | Motion sim controller |
| PC2 (Rig B) | 192.168.4.20 | Bridge + game |
| ESP32-C6 Rig B | 192.168.4.120 | Motion sim controller |
| Future OpenCV box | 192.168.4.200 | Reserved |

### Bench Test: UDP Latency Baseline
Before any motor code, verify the wireless link:

1. **Flash ESP32-C6** with UDP echo server (port 20777)
2. **PC bridge** sends timestamped packet, measures round-trip
3. **Target:** <5 ms one-way, <2 ms jitter, 0% loss at 60 Hz for 5 minutes
4. **If failing:** Move to AP mode on ESP32-C6 (PC connects directly), retest

---

## Phase 1: PC Bridge Software (Week 1–2)

### Architecture
```
Dirt Rally 2.0 → UDP 20777 → PC Bridge → WiFi UDP → ESP32-C6 → IBT-2 → Motor
```

### Bridge Responsibilities
| Function | Details |
|----------|---------|
| **Read DR2.0 telemetry** | UDP 20777, 60 Hz, binary struct |
| **Map telemetry → motor commands** | Sway/surge/heave → 3 motor PWM values |
| **Transmit** | UDP to ESP32-C6 at 60 Hz, no buffering |
| **Dead reckoning** | If packet missed, interpolate for 1 frame (16 ms) then hold last |

### DR2.0 Telemetry Fields You Need
From the `PacketMotionData` / `PacketCarTelemetryData` (depends on which API you're tapping):

| Field | Motor Mapping |
|-------|---------------|
| `gForceLateral` (sway/left-right) | Motor 1 (roll) |
| `gForceLongitudinal` (surge/brake-accel) | Motor 2 (pitch) |
| `suspensionPosition[4]` averaged or `worldPositionY` derivative | Motor 3 (heave/bounce) |
| `speed` | Scaling factor for intensity |

### Bridge Implementation Notes
- **Language:** C++ (Windows) or Python with `asyncio` + `socket` (faster to prototype)
- **Socket flags:** `SO_SNDBUF` = small (1–2 packets), `MSG_DONTWAIT`, disable Nagle
- **Thread priority:** `THREAD_PRIORITY_TIME_CRITICAL` (Win32) or `SCHED_FIFO` (Linux)
- **No GUI on hot path:** Headless or UI on separate thread with queue

---

## Phase 2: ESP32-C6 Firmware (Week 2–3)

### Architecture
```
WiFi UDP RX Task (Core 0) → Telemetry Queue → Motor Control Task (Core 1) → LEDC → IBT-2
```

### Task Breakdown
| Task | Core | Priority | Function |
|------|------|----------|----------|
| `wifi_rx_task` | 0 | 24 (high) | Block on `recvfrom()`, parse packet, push to queue |
| `motor_control_task` | 1 | 24 (high) | Pop queue, compute PWM, update LEDC |
| `watchdog_task` | 1 | 5 | Monitor packet age, safety stop if stale >100 ms |

### Key Implementation Details

**WiFi Setup:**
```c
// ESP32-C6 as STA to dedicated router
wifi_config_t sta_config = {
    .ssid = "MOTION_SIM_5G",
    .password = "yourpassword",
    .threshold.authmode = WIFI_AUTH_WPA3_PSK,
};
// Force 802.11n/ac on 5 GHz, disable power save
esp_wifi_set_ps(WIFI_PS_NONE);
```

**UDP Socket:**
```c
// Non-blocking, small buffer, single destination
int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
int sndbuf = 0;  // minimal
setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
// Bind to port 20777
```

**Packet Format (PC → ESP32):**
Keep it tight — 12 bytes minimum:

```c
typedef struct __attribute__((packed)) {
    uint32_t seq;           // Sequence number for loss detection
    int16_t  motor1_duty;   // -1023 to +1023 (signed, center = 0)
    int16_t  motor2_duty;
    int16_t  motor3_duty;
    uint16_t checksum;      // Simple XOR or CRC-16
} motion_cmd_t;
```

**Motor Control Logic:**
```c
// Per motor: signed duty → direction + magnitude
void motor_set(int motor_id, int16_t duty) {
    // duty > 0: forward, duty < 0: reverse, 0: stop
    // Apply dead-time on direction change
    // Update LEDC duty
}
```

**Safety:**
- **Packet timeout:** No valid packet for >100 ms → all motors to 0 (coast or brake)
- **Duty clamping:** Hard limit at 80% max (819/1023) to prevent driver overcurrent
- **Thermal:** Optional — read IBT-2 current sense (R_IS/L_IS) via ADC, throttle if hot

---

## Phase 3: 3DOF Motion Math (Week 3–4)

### Rig Geometry
For each rig, you have 3 motors. Typical 3DOF platform layout:

| Motor | Position | DOF |
|-------|----------|-----|
| Motor 1 | Front left | Roll + Heave |
| Motor 2 | Front right | Roll + Heave |
| Motor 3 | Rear center | Pitch + Heave |

### Inverse Kinematics (simplified)
Given telemetry `gForceLateral`, `gForceLongitudinal`, `heave`:

```
roll  = gForceLateral    * GAIN_ROLL
pitch = gForceLongitudinal * GAIN_PITCH
heave = suspension_avg  * GAIN_HEAVE

motor1 =  heave + roll
motor2 =  heave - roll
motor3 =  heave + pitch
```

This runs on the **PC bridge** (more CPU headroom), ESP32 just receives final motor duties.

### Tuning Parameters
| Parameter | Start Value | Tuning |
|-----------|-------------|--------|
| `GAIN_ROLL` | 200 | Increase until cornering feels "weighty" |
| `GAIN_PITCH` | 250 | Braking dive / acceleration squat |
| `GAIN_HEAVE` | 150 | Road texture, bumps |
| `SMOOTHING` | 0.3 (EMA alpha) | Lower = smoother, higher = more responsive |
| `DEADZONE` | 5% | Ignore tiny telemetry noise |

---

## Phase 4: Integration & Tuning (Week 4–5)

### Per-Rig Checklist
- [ ] Router powered, both rigs on 5 GHz, static IPs confirmed
- [ ] PC bridge running, DR2.0 telemetry flowing, UDP packets reaching ESP32
- [ ] ESP32-C6 receiving at 60 Hz, <5 ms jitter verified with logic analyzer or LED toggle
- [ ] All 3 motors responding correctly to each DOF (test one at a time)
- [ ] No packet loss during 30-minute race
- [ ] Safety timeout working (pull WiFi, motors stop within 100 ms)

### Two-Rig Coordination (Pre-OpenCV)
Even before the sync TV, ensure:
- Both rigs on **different 5 GHz channels** or same channel with enough airtime — test with both running simultaneously
- Router QoS gives both rigs equal priority
- No crosstalk: PC1 only talks to ESP32-A, PC2 to ESP32-B (destination IP check)

---

## Phase 5: OpenCV Sync Machine (Future)

### Concept
```
PC1 telemetry ─┐
               ├──→ OpenCV Box (192.168.4.200) → Parse position → Overlay on TV
PC2 telemetry ─┘
```

### Data Source
Instead of tapping UDP again, have the PC bridges **also** emit a small summary packet to the OpenCV box:
```c
typedef struct {
    uint32_t lap_time_ms;
    float    distance;      // meters into stage
    float    speed;
    uint8_t  gear;
    uint16_t position;      // overall rank if available
} race_state_t;
```

### OpenCV Overlay
- Read both race states
- Draw split-screen or overlay comparison
- Update at 30 Hz (display rate), no need for 60 Hz

---

## Hardware Shopping / Verification List

| Item | Status | Note |
|------|--------|------|
| 2× ESP32-C6-DevKitC-1 | ☐ | Confirm 5 GHz support, camera flag antenna |
| 2× IBT-2 per rig = 6 total | ✅ | Already proven |
| 3× 24V 250W motor per rig = 6 total | ✅ | Already proven |
| 2× LRS-350-24 per rig = 6 total | ✅ | Already proven |
| Portable router (WiFi 6, 5 GHz) | ☐ | ASUS AX57, GL.iNet Beryl, or similar |
| Ethernet cables (PC → router) | ☐ | Hardwire PCs if possible, even if ESP is wireless |

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| WiFi dropout mid-race | Safety timeout on ESP32, auto-reconnect, bridge buffers last 3 frames |
| Both rigs on same channel saturate | Use 40 MHz + different primary channels, or router with dual 5 GHz |
| DR2.0 telemetry format changes | Read from shared memory (CrewChief, SimHub) instead of UDP if needed |
| Motor mechanical resonance | Notch filter in PC bridge at platform natural frequency (~8–12 Hz typical) |
