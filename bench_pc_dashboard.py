#!/usr/bin/env python3
"""
===============================================================================
 MotionSim High-Tech Cyberpunk PC Benchmark Dashboard & Diagnostics System
===============================================================================
 Runs on Gaming PC:
  - UDP 20778: High-precision 60 Hz benchmark pinger to ESP32
  - UDP 20777: Dirt Rally 2.0 Telemetry receiver & parser
  - HTTP 8080: Futuristic Cyberpunk Web UI with live real-time plots & API controls
===============================================================================
"""

import sys
import os
import time
import math
import socket
import struct
import threading
import json
import csv
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

# --- CONSTANTS & CONFIG ---
UDP_BENCH_PORT = 20778
UDP_DR2_PORT   = 20777
HTTP_PORT      = 8080

MAGIC = 0x4D53  # "MS"

# Global Shared State
state_lock = threading.Lock()
target_ip = "192.168.1.110"  # default target ESP32 IP
ping_rate_hz = 60
bench_active = True
dr2_active = False

# Rolling Metrics
history_rtt_ms = []
history_dr2_sway = []
history_dr2_surge = []
history_dr2_heave = []
history_max_len = 120  # 2 seconds of 60Hz history for fast UI charts

bench_metrics = {
    "total_sent": 0,
    "total_recv": 0,
    "lost": 0,
    "out_of_order": 0,
    "duplicates": 0,
    "rtt_last_ms": 0.0,
    "rtt_avg_ms": 0.0,
    "rtt_min_ms": 999.0,
    "rtt_max_ms": 0.0,
    "rtt_p95_ms": 0.0,
    "rtt_p99_ms": 0.0,
    "jitter_ms": 0.0,
    "packet_rate_hz": 0.0,
    "esp_max_stall_us": 0,
    "esp_rssi": 0,
    "esp_ip": target_ip,
}

dr2_metrics = {
    "connected": False,
    "speed_kph": 0.0,
    "gear": 0,
    "rpm": 0,
    "sway_g": 0.0,
    "surge_g": 0.0,
    "heave_g": 0.0,
    "last_packet_t": 0.0
}

# --- UDP BENCHMARK THREAD ---
def bench_pinger_thread():
    global target_ip, ping_rate_hz, bench_active
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.1)

    seq = 0
    recent_rtts = []

    print(f"[BENCH] UDP Pinger started target={target_ip}:{UDP_BENCH_PORT} rate={ping_rate_hz}Hz")

    while True:
        if not bench_active or not target_ip:
            time.sleep(0.1)
            continue

        interval = 1.0 / ping_rate_hz
        start_t = time.perf_counter()

        seq = (seq + 1) & 0xFFFFFFFF
        pc_ts_us = int(time.time_ns() // 1000) & 0xFFFFFFFF

        # Pack 16-byte bench_packet_t (magic, flags, seq, pc_ts_us, esp_ts_us)
        pkt = struct.pack("<HHIII", MAGIC, 0, seq, pc_ts_us, 0)

        try:
            sock.sendto(pkt, (target_ip, UDP_BENCH_PORT))

            with state_lock:
                bench_metrics["total_sent"] += 1

            # Wait for echo
            try:
                data, addr = sock.recvfrom(64)
                recv_t_us = int(time.time_ns() // 1000) & 0xFFFFFFFF

                if len(data) == 16:
                    r_magic, r_flags, r_seq, r_pc_ts, r_esp_ts = struct.unpack("<HHIII", data)
                    if r_magic == MAGIC:
                        rtt_us = (recv_t_us - r_pc_ts) & 0xFFFFFFFF
                        rtt_ms = rtt_us / 1000.0

                        with state_lock:
                            bench_metrics["total_recv"] += 1
                            bench_metrics["rtt_last_ms"] = round(rtt_ms, 2)
                            if rtt_ms < bench_metrics["rtt_min_ms"]: bench_metrics["rtt_min_ms"] = round(rtt_ms, 2)
                            if rtt_ms > bench_metrics["rtt_max_ms"]: bench_metrics["rtt_max_ms"] = round(rtt_ms, 2)

                            history_rtt_ms.append(round(rtt_ms, 2))
                            if len(history_rtt_ms) > history_max_len:
                                history_rtt_ms.pop(0)

                            recent_rtts.append(rtt_ms)
                            if len(recent_rtts) > 300: # 5 sec window
                                recent_rtts.pop(0)

                            if len(recent_rtts) > 0:
                                bench_metrics["rtt_avg_ms"] = round(sum(recent_rtts) / len(recent_rtts), 2)
                                sorted_rtts = sorted(recent_rtts)
                                idx95 = int(len(sorted_rtts) * 0.95)
                                idx99 = int(len(sorted_rtts) * 0.99)
                                bench_metrics["rtt_p95_ms"] = round(sorted_rtts[min(idx95, len(sorted_rtts)-1)], 2)
                                bench_metrics["rtt_p99_ms"] = round(sorted_rtts[min(idx99, len(sorted_rtts)-1)], 2)

                                # Jitter (std dev)
                                avg = bench_metrics["rtt_avg_ms"]
                                variance = sum((x - avg) ** 2 for x in recent_rtts) / len(recent_rtts)
                                bench_metrics["jitter_ms"] = round(math.sqrt(variance), 2)

            except socket.timeout:
                with state_lock:
                    bench_metrics["lost"] += 1

        except Exception as e:
            time.sleep(0.1)

        # High-precision pacing
        elapsed = time.perf_counter() - start_t
        rem = interval - elapsed
        if rem > 0:
            time.sleep(rem)

# --- DIRT RALLY 2.0 TELEMETRY THREAD ---
def dr2_telemetry_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", UDP_DR2_PORT))
        print(f"[DR2] Telemetry listener bound to port {UDP_DR2_PORT}")
    except Exception as e:
        print(f"[DR2] Port {UDP_DR2_PORT} bind warning: {e}")
        return

    sock.settimeout(0.5)

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            # DR2.0 UDP packet parsing (66 floats array)
            if len(data) >= 264:
                floats = struct.unpack(f"<{len(data)//4}f", data[:(len(data)//4)*4])
                speed = floats[7] * 3.6  # m/s to kph
                sway = floats[31]        # lateral g
                surge = floats[32]       # longitudinal g
                heave = floats[33]       # vertical g
                gear = int(floats[37])
                rpm = int(floats[38] * 10)

                with state_lock:
                    dr2_metrics["connected"] = True
                    dr2_metrics["speed_kph"] = round(speed, 1)
                    dr2_metrics["gear"] = gear
                    dr2_metrics["rpm"] = rpm
                    dr2_metrics["sway_g"] = round(sway, 2)
                    dr2_metrics["surge_g"] = round(surge, 2)
                    dr2_metrics["heave_g"] = round(heave, 2)
                    dr2_metrics["last_packet_t"] = time.time()

                    history_dr2_sway.append(round(sway, 2))
                    history_dr2_surge.append(round(surge, 2))
                    history_dr2_heave.append(round(heave, 2))
                    if len(history_dr2_sway) > history_max_len: history_dr2_sway.pop(0)
                    if len(history_dr2_surge) > history_max_len: history_dr2_surge.pop(0)
                    if len(history_dr2_heave) > history_max_len: history_dr2_heave.pop(0)
        except socket.timeout:
            with state_lock:
                if time.time() - dr2_metrics["last_packet_t"] > 2.0:
                    dr2_metrics["connected"] = False
        except Exception as e:
            time.sleep(0.1)

# --- HTTP DASHBOARD SERVER ---
class CyberpunkDashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return  # Silence HTTP request logging for clean terminal output

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        if path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_DASHBOARD.encode('utf-8'))
        elif path == "/api/data":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            with state_lock:
                data = {
                    "bench": bench_metrics,
                    "dr2": dr2_metrics,
                    "history_rtt": history_rtt_ms,
                    "history_sway": history_dr2_sway,
                    "history_surge": history_dr2_surge,
                    "history_heave": history_dr2_heave,
                    "target_ip": target_ip,
                    "ping_rate_hz": ping_rate_hz
                }
            self.wfile.write(json.dumps(data).encode('utf-8'))
        elif path == "/api/config":
            global target_ip, ping_rate_hz
            if "ip" in qs: target_ip = qs["ip"][0]
            if "rate" in qs: ping_rate_hz = int(qs["rate"][0])
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        else:
            self.send_response(404)
            self.end_headers()

# --- CYBERPUNK HTML WEB DASHBOARD ---
HTML_DASHBOARD = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>CYBERPUNK MOTIONSIM BENCHMARK DASHBOARD</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    :root {
      --bg: #07090e;
      --card-bg: rgba(18, 24, 38, 0.85);
      --accent-cyan: #00f0ff;
      --accent-magenta: #ff0055;
      --accent-green: #00ff66;
      --accent-yellow: #ffcc00;
      --text: #e2e8f0;
      --dim: #64748b;
    }
    body {
      background: var(--bg);
      color: var(--text);
      font-family: 'Consolas', 'Courier New', monospace;
      margin: 0;
      padding: 20px;
    }
    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 2px solid var(--accent-cyan);
      padding-bottom: 12px;
      margin-bottom: 20px;
    }
    h1 {
      margin: 0;
      font-size: 1.8rem;
      color: var(--accent-cyan);
      text-transform: uppercase;
      letter-spacing: 3px;
      text-shadow: 0 0 12px rgba(0, 240, 255, 0.5);
    }
    .status-pill {
      background: var(--card-bg);
      border: 1px solid var(--accent-cyan);
      padding: 6px 14px;
      border-radius: 4px;
      font-size: 0.9rem;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 15px;
      margin-bottom: 20px;
    }
    .card {
      background: var(--card-bg);
      border: 1px solid #1e293b;
      border-radius: 8px;
      padding: 16px;
      box-shadow: 0 4px 20px rgba(0,0,0,0.6);
    }
    .card h3 {
      margin: 0 0 8px 0;
      font-size: 0.8rem;
      color: var(--dim);
      text-transform: uppercase;
    }
    .metric {
      font-size: 2rem;
      font-weight: bold;
    }
    .unit { font-size: 0.9rem; color: var(--dim); }
    .charts-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 20px;
      margin-bottom: 20px;
    }
    canvas {
      width: 100% !important;
      max-height: 250px;
    }
    .controls {
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
    }
    button {
      background: var(--card-bg);
      border: 1px solid var(--accent-cyan);
      color: var(--accent-cyan);
      padding: 10px 18px;
      font-weight: bold;
      border-radius: 4px;
      cursor: pointer;
      font-family: inherit;
      transition: all 0.2s;
    }
    button:hover {
      background: var(--accent-cyan);
      color: var(--bg);
      box-shadow: 0 0 15px rgba(0,240,255,0.7);
    }
    input {
      background: var(--card-bg);
      border: 1px solid var(--dim);
      color: #fff;
      padding: 8px;
      border-radius: 4px;
      font-family: inherit;
    }
  </style>
</head>
<body>

  <div class="header">
    <h1>⚡ MOTIONSIM HIGH-TECH BENCHMARK DASHBOARD</h1>
    <div class="status-pill" id="target-info">TARGET: 192.168.1.110 @ 60Hz</div>
  </div>

  <div class="grid">
    <div class="card">
      <h3>Avg RTT Latency</h3>
      <div class="metric" style="color:var(--accent-cyan)" id="rtt-avg">0.0 <span class="unit">ms</span></div>
    </div>
    <div class="card">
      <h3>P99 Peak Latency</h3>
      <div class="metric" style="color:var(--accent-yellow)" id="rtt-p99">0.0 <span class="unit">ms</span></div>
    </div>
    <div class="card">
      <h3>Jitter (Std Dev)</h3>
      <div class="metric" style="color:var(--accent-green)" id="jitter">0.0 <span class="unit">ms</span></div>
    </div>
    <div class="card">
      <h3>Packet Loss</h3>
      <div class="metric" style="color:var(--accent-magenta)" id="loss">0</div>
    </div>
    <div class="card">
      <h3>Dirt Rally 2.0 Speed</h3>
      <div class="metric" style="color:var(--accent-cyan)" id="dr2-speed">0.0 <span class="unit">kph</span></div>
    </div>
  </div>

  <div class="charts-grid">
    <div class="card">
      <h3>Real-Time RTT Latency (ms)</h3>
      <canvas id="rttChart"></canvas>
    </div>
    <div class="card">
      <h3>Dirt Rally 2.0 G-Forces (Sway / Surge / Heave)</h3>
      <canvas id="dr2Chart"></canvas>
    </div>
  </div>

  <div class="card">
    <h3>Target Controls & Remote ESP32 API</h3>
    <div class="controls">
      <input type="text" id="ip-in" value="192.168.1.110" placeholder="ESP32 IP">
      <input type="number" id="rate-in" value="60" min="10" max="500" placeholder="Hz">
      <button onclick="updateConfig()">Set Target/Hz</button>
      <button onclick="sendEspOled('stats')">OLED: Stats</button>
      <button onclick="sendEspOled('graph')">OLED: Sparkline</button>
      <button onclick="sendEspOled('wifi')">OLED: WiFi Info</button>
      <button onclick="sendEspOled('eyes')">OLED: Eyes</button>
      <button onclick="sendEspOled('cycle')">OLED: Cycle</button>
    </div>
  </div>

  <script>
    const ctxRtt = document.getElementById('rttChart').getContext('2d');
    const chartRtt = new Chart(ctxRtt, {
      type: 'line',
      data: { labels: Array(120).fill(''), datasets: [{ label: 'RTT Latency (ms)', data: [], borderColor: '#00f0ff', borderWidth: 2, pointRadius: 0, fill: false }] },
      options: { scales: { y: { beginAtZero: true, grid: { color: '#1e293b' } }, x: { display: false } }, animation: false }
    });

    const ctxDr2 = document.getElementById('dr2Chart').getContext('2d');
    const chartDr2 = new Chart(ctxDr2, {
      type: 'line',
      data: {
        labels: Array(120).fill(''),
        datasets: [
          { label: 'Sway G', data: [], borderColor: '#ff0055', borderWidth: 2, pointRadius: 0 },
          { label: 'Surge G', data: [], borderColor: '#00ff66', borderWidth: 2, pointRadius: 0 },
          { label: 'Heave G', data: [], borderColor: '#ffcc00', borderWidth: 2, pointRadius: 0 }
        ]
      },
      options: { scales: { y: { grid: { color: '#1e293b' } }, x: { display: false } }, animation: false }
    });

    async function poll() {
      try {
        const res = await fetch('/api/data');
        const d = await res.json();

        document.getElementById('rtt-avg').innerHTML = d.bench.rtt_avg_ms + ' <span class="unit">ms</span>';
        document.getElementById('rtt-p99').innerHTML = d.bench.rtt_p99_ms + ' <span class="unit">ms</span>';
        document.getElementById('jitter').innerHTML = d.bench.jitter_ms + ' <span class="unit">ms</span>';
        document.getElementById('loss').innerHTML = d.bench.lost;
        document.getElementById('dr2-speed').innerHTML = d.dr2.speed_kph + ' <span class="unit">kph (G:' + d.dr2.gear + ')</span>';
        document.getElementById('target-info').innerText = 'TARGET: ' + d.target_ip + ' @ ' + d.ping_rate_hz + 'Hz';

        chartRtt.data.datasets[0].data = d.history_rtt;
        chartRtt.update();

        chartDr2.data.datasets[0].data = d.history_sway;
        chartDr2.data.datasets[1].data = d.history_surge;
        chartDr2.data.datasets[2].data = d.history_heave;
        chartDr2.update();
      } catch(e){}
    }

    async function updateConfig() {
      const ip = document.getElementById('ip-in').value;
      const rate = document.getElementById('rate-in').value;
      await fetch('/api/config?ip=' + ip + '&rate=' + rate);
    }

    async function sendEspOled(mode) {
      const ip = document.getElementById('ip-in').value;
      try {
        await fetch('http://' + ip + '/api/oled?mode=' + mode);
      } catch(e) { alert('Failed to reach ESP32 at ' + ip); }
    }

    setInterval(poll, 250);
  </script>
</body>
</html>
"""

def main():
    threading.Thread(target=bench_pinger_thread, daemon=True).start()
    threading.Thread(target=dr2_telemetry_thread, daemon=True).start()

    server = HTTPServer(('0.0.0.0', HTTP_PORT), CyberpunkDashboardHandler)
    print(f"\n==========================================================================")
    print(f" 🚀 MOTIONSIM HIGH-TECH CYBERPUNK BENCHMARK DASHBOARD RUNNING AT:")
    print(f" 👉 http://localhost:{HTTP_PORT}")
    print(f"==========================================================================\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[BENCH] Stopping dashboard server.")

if __name__ == "__main__":
    main()
