# MotionSim Testing & Telemetry Guide

This guide covers how to configure DiRT Rally for telemetry output, how to test the network/hardware latency using `motionsimbench`, and how to perform hardware-in-the-loop tests with `motionsimbot`.

---

## 🏎️ 1. DiRT Rally Telemetry Configuration

To allow the MotionSim rig (or the local UDP Viewer) to receive physics data from DiRT Rally, you must enable UDP telemetry in the game's configuration file.

### Editing `hardware_settings_config.xml`
1. Navigate to your DiRT Rally settings folder. Typically found at:
   `Documents\My Games\DiRT Rally 2.0\hardwaresettings\hardware_settings_config.xml`
2. Open the file in a text editor and locate the `<motion_platform>` block.
3. Modify the `<udp>` tag as follows:

```xml
<motion_platform>
    <dbox enabled="true" />
    <udp enabled="true" extradata="3" ip="127.0.0.1" port="20777" delay="1" />
    <custom_udp enabled="false" filename="packet_data.xml" ip="127.0.0.1" port="20777" delay="1" />
    <fanatec enabled="true" pedalVibrationScale="1.0" wheelVibrationScale="1.0" ledTrueForGearsFalseForSpeed="true" />
</motion_platform>
```

> [!IMPORTANT]
> - `extradata="3"` is **crucial**! It instructs the game to send the richer telemetry packet format (containing suspension data, g-forces, and more) required by the motion platform.
> - `ip="127.0.0.1"` sends data to your local PC. When you are ready to send data directly to the physical rig, change this IP to the ESP32's IP address (which is shown on the ESP32's OLED screen when connected to WiFi).
> - `port="20777"` is the default port we will listen on.

---

## 🧪 2. Testing with `motionsimbench`

`motionsimbench` is used to validate your WiFi and PC-to-ESP32 UDP latency before running actual hardware. 

### Running the Test
1. Connect your ESP32 flashed with `motionsimbench` to a power source. Note the IP address displayed on the OLED screen.
2. Ensure your PC is connected to the same network (preferably 5GHz WiFi or Ethernet for the PC).
3. On your PC, open a terminal in `motionsim/motionsimbench` and run:
   ```bash
   python bench_pc_dashboard.py
   ```
4. Open the PC Dashboard at `http://localhost:8080`.
5. Start DiRT Rally and enter a stage.
6. Verify the telemetry values (Sway, Surge, Heave) on the PC Dashboard and monitor the UDP **RTT latency** graph. Ensure the `p99` latency is under 8-10ms and packet loss is 0%.

---

## ⚙️ 3. Testing `motionsimbot` with Hardware

Once latency is verified, you can test the production firmware (`motionsimbot`) connected to the physical motors.

1. **Safety First**: Ensure the rig is clear of obstacles and people. Keep your hand near the physical **Emergency Stop** button.
2. **Update Game Config**: If you haven't already, change the `ip="..."` in `hardware_settings_config.xml` to match the IP of the `motionsimbot` ESP32.
3. **Power On**: Power up the 24V motor battery bank and the ESP32. Wait for the OLED to display the IP and confirm "UDP Mode".
4. **Calibration**: If this is the first run, use the REST API to calibrate the physical zero points:
   ```bash
   curl -X POST http://<ESP32_IP>/api/calibrate?motor_id=1
   ```
5. **Start Telemetry**: Launch DiRT Rally. As soon as the stage loads and you accelerate, the rig should react to the pitch and roll data.
6. **PID Tuning**: Use the OLED or REST API (`/api/stats` and `/api/pid`) to observe target vs. actual angles and tune KP/KI/KD on the fly for responsive but smooth movement.

---

## 📡 Local UDP Viewer App

A local UDP Viewer web application has been provided to inspect the telemetry packets visually before sending them to the ESP32. 
To use it:
1. Keep the `ip="127.0.0.1"` in the DiRT config.
2. Start the UDP Viewer app.
3. Observe real-time G-forces, RPM, Speed, and raw packet bytes in the browser.
