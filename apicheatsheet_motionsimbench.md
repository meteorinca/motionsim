# MotionSim Benchmarking API Cheat Sheet & Wiring Guide

This document lists the hardware wiring, build instructions, REST API commands, UDP protocol specifications, and testing protocols for the `motionsimbench` firmware and PC Cyberpunk Dashboard.

---

## 🔌 Hardware Wiring Guide (ESP32-C3 SuperMini)

| Component | Component Pin | ESP32-C3 SuperMini Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Built-in LED** | — | `GPIO 8` | On-board status LED (Active LOW) |
| **Passive Buzzer** | Signal (+) | `GPIO 3` | Audio tones & alerts |
| | GND (-) | `GND` | Common Ground |
| **OLED Display (SSD1306)** | SDA | `GPIO 7` | I2C Data (400 kHz) |
| | SCL | `GPIO 6` | I2C Clock |
| | VCC | `3.3V` | 3.3V Power |
| | GND | `GND` | Common Ground |
| **Green External LED** | Anode (+) | `GPIO 20` | Via 220Ω resistor to GND |
| **Red External LED** | Anode (+) | `GPIO 21` | Via 220Ω resistor to GND |
| **Button 1** | Terminal | `GPIO 0` | Cycle OLED Screen Mode |
| **Button 2** | Terminal | `GPIO 1` | Reset Rolling Bench Statistics |
| **Boot Button** | — | `GPIO 9` | On-board BOOT button (Hold 7s to reset WiFi) |

---

## 🛠️ Build & Flash Instructions

### 1. Building for ESP32-C3 (Default Target)
Open an ESP-IDF terminal in `motionsim/motionsimbench`:

```bash
cd motionsimbench
idf.py -DBOARD=esp32c3_bench set-target esp32c3 build
```

### 2. Flashing to ESP32-C3
```bash
idf.py -p COMx flash monitor
```
*(Replace `COMx` with your actual serial port)*

### 3. Switching Target to ESP32-C6 / ESP32-S3 (Future Hardware)
When ESP32-C6 or S3 arrives:
```bash
idf.py -DBOARD=esp32c6_bench set-target esp32c6 fullclean build
```

---

## 🌐 HTTP GET/POST REST API Reference

Base URL: `http://motionsimbench1.local:80` (or IP address shown on OLED)

| Endpoint | Method | Description | Example Response / Usage |
|---|---|---|---|
| `/api/stats` | `GET` | Get live UDP benchmarking metrics, RSSI, max stall delay, and OLED mode | `{"rx_count":120,"lost_count":0,"max_rx_interval_us":16420,"pkt_rate_hz":60.0,"rssi":-45}` |
| `/api/oled` | `GET` | Change OLED screen mode (`stats`, `graph`, `wifi`, `eyes`, `cycle`) | `GET /api/oled?mode=graph` |
| `/api/reset` | `GET` | Reset rolling statistics and lost packet counters | `GET /api/reset` |
| `/api/reboot` | `GET` | Reboot the ESP32 benchmark device | `GET /api/reboot` |
| `/wifi` | `GET` | Get saved WiFi networks and SoftAP fallback state | `GET /wifi` |
| `/wifi` | `POST` | Save new WiFi credentials to NVS and reboot | `POST /wifi` Body: `{"ssid":"MOTION_5G","pass":"secret"}` |
| `/ota` | `POST` | Stream firmware binary (`motionsimbench.bin`) for OTA update | `POST /ota` with raw binary body |

---

## 🛰️ UDP Benchmark Protocol Specification

- **Port**: `20778` (UDP)
- **Rate**: Default `60 Hz` (16.66 ms interval)
- **Packet Structure** (16 bytes packed, little-endian):

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;         // 0x4D53 ("MS")
    uint16_t flags;         // Reserved / test mode flags
    uint32_t seq;           // Packet sequence number
    uint32_t pc_ts_us;      // PC send timestamp (microseconds)
    uint32_t esp_ts_us;     // ESP32 receive timestamp (filled on echo)
} bench_packet_t;
```

---

## 💻 PC Cyberpunk Dashboard Application

Launch the high-tech PC dashboard and telemetry listener on your Gaming PC:

```bash
python bench_pc_dashboard.py
```

Open browser at **`http://localhost:8080`**.

Features:
- Real-time RTT Latency gauge and line chart (p50, p90, p95, p99, jitter).
- Live Dirt Rally 2.0 telemetry visualization (Sway, Surge, Heave G-forces, Speed, Gear).
- Interactive ESP32 control buttons.
- Target IP & ping rate configuration.

---

## 🧪 The 5 Exhaustive Test Cases Protocol

1. **Test 1: Baseline Idle** — Run `bench_pc_dashboard.py` at 60 Hz for 5 minutes without game running. Target: p99 ≤ 8 ms, 0% loss.
2. **Test 2: Under Dirt Rally 2.0 Load** — Drive a stage in DR2.0 with benchmark active. Verify GPU/CPU load doesn't spike latency.
3. **Test 3: Rate Saturation** — Increase ping rate to 500 Hz on dashboard to find link saturation knee.
4. **Test 4: Safety Auto-Stop** — Stop PC script; verify ESP32 OLED displays `SAFETY STOP` within 100 ms.
5. **Test 5: Dual Device Simultaneous** — Run two ESP32s on the same WiFi network to verify zero crosstalk.
