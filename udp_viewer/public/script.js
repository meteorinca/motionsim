const socket = io();

// DOM Elements
const statusIndicator = document.getElementById('connection-dot');
const statusText = document.getElementById('connection-status');
const btnToggleRaw = document.getElementById('btn-toggle-raw');
const dashboardView = document.getElementById('dashboard-view');
const rawView = document.getElementById('raw-view');

// Metrics DOM
const valSpeed = document.getElementById('val-speed');
const valGear = document.getElementById('val-gear');
const valRpm = document.getElementById('val-rpm');
const barRpm = document.getElementById('bar-rpm');

const valSway = document.getElementById('val-sway');
const valHeave = document.getElementById('val-heave');
const valSurge = document.getElementById('val-surge');
const gDot = document.getElementById('g-dot');

const suspFL = document.getElementById('susp-fl');
const suspFR = document.getElementById('susp-fr');
const suspRL = document.getElementById('susp-rl');
const suspRR = document.getElementById('susp-rr');
const valSuspFL = document.getElementById('val-susp-fl');
const valSuspFR = document.getElementById('val-susp-fr');
const valSuspRL = document.getElementById('val-susp-rl');
const valSuspRR = document.getElementById('val-susp-rr');

const valPitch = document.getElementById('val-pitch');
const valRoll = document.getElementById('val-roll');
const valYaw = document.getElementById('val-yaw');

const rawOutput = document.getElementById('raw-output');
const rawStats = document.getElementById('raw-stats');

let isRawView = false;
const MAX_RPM = 8000; // arbitrary max rpm for bar

// Toggle View
btnToggleRaw.addEventListener('click', () => {
    isRawView = !isRawView;
    if (isRawView) {
        btnToggleRaw.textContent = 'Show Dashboard';
        dashboardView.classList.remove('active');
        rawView.classList.add('active');
    } else {
        btnToggleRaw.textContent = 'Enable RAW View';
        rawView.classList.remove('active');
        dashboardView.classList.add('active');
    }
});

// Socket Events
socket.on('connect', () => {
    statusIndicator.className = 'dot connected';
    statusText.textContent = 'Listening on UDP 20777';
});

socket.on('disconnect', () => {
    statusIndicator.className = 'dot disconnected';
    statusText.textContent = 'Disconnected from Backend';
});

// Helper to extract zero-copy DataView from ArrayBuffer, Buffer attachment, or Uint8Array
function getPacketView(packet) {
    if (!packet) return null;
    const raw = packet.buffer || packet.data || packet;
    if (raw instanceof ArrayBuffer) {
        return new DataView(raw);
    } else if (ArrayBuffer.isView(raw)) {
        return new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
    } else if (Array.isArray(raw)) {
        const u8 = new Uint8Array(raw);
        return new DataView(u8.buffer);
    }
    return null;
}

// Telemetry Event (Zero-Copy Non-Blocking)
socket.on('telemetry', (packet) => {
    const view = getPacketView(packet);
    if (!view) return;

    // Update RAW view if active
    if (isRawView) {
        rawStats.textContent = `${view.byteLength} bytes from ${packet.address || 'local'}:${packet.port || 20777}`;
        let hex = '';
        const u8 = new Uint8Array(view.buffer, view.byteOffset, view.byteLength);
        for (let i = 0; i < u8.length; i++) {
            hex += u8[i].toString(16).padStart(2, '0') + ' ';
            if ((i + 1) % 16 === 0) hex += '\n';
        }
        rawOutput.textContent = hex;
        return;
    }

    // Parse Extradata=3 (Common Codemasters Format, 256+ bytes, 64+ floats)
    if (view.byteLength >= 256) {
        // Speed (byte 28)
        const speedMS = view.getFloat32(28, true);
        const speedKMH = Math.max(0, speedMS * 3.6);
        valSpeed.textContent = speedKMH.toFixed(0);

        // Gear (byte 132)
        const gearFloat = view.getFloat32(132, true);
        let gearStr = "N";
        if (gearFloat === 0) gearStr = "R";
        else if (gearFloat > 1) gearStr = Math.round(gearFloat - 1).toString();
        valGear.textContent = gearStr;

        // RPM (byte 148)
        const rpm = (view.byteLength >= 152) ? view.getFloat32(148, true) * 10 : 0;
        valRpm.textContent = rpm.toFixed(0);
        let rpmPct = Math.min(100, Math.max(0, (rpm / MAX_RPM) * 100));
        barRpm.style.width = `${rpmPct}%`;

        // Acceleration / G-Forces (bytes 120, 124, 128)
        const sway = view.getFloat32(120, true);
        const heave = view.getFloat32(124, true);
        const surge = view.getFloat32(128, true);
        
        valSway.textContent = sway.toFixed(2);
        valHeave.textContent = heave.toFixed(2);
        valSurge.textContent = surge.toFixed(2);

        // Update G-Dot
        const maxG = 2.0;
        let dotX = (sway / maxG) * 50 + 50;
        let dotY = (surge / maxG) * 50 + 50;
        dotX = Math.max(0, Math.min(100, dotX));
        dotY = Math.max(0, Math.min(100, dotY));
        gDot.style.left = `${dotX}%`;
        gDot.style.top = `${dotY}%`;

        // Suspension Positions (RL:68, RR:72, FL:76, FR:80)
        const susp_rl = view.getFloat32(68, true);
        const susp_rr = view.getFloat32(72, true);
        const susp_fl = view.getFloat32(76, true);
        const susp_fr = view.getFloat32(80, true);

        valSuspFL.textContent = susp_fl.toFixed(2);
        valSuspFR.textContent = susp_fr.toFixed(2);
        valSuspRL.textContent = susp_rl.toFixed(2);
        valSuspRR.textContent = susp_rr.toFixed(2);

        const maxSusp = 0.5;
        suspFL.style.height = `${Math.min(100, (susp_fl / maxSusp) * 100)}%`;
        suspFR.style.height = `${Math.min(100, (susp_fr / maxSusp) * 100)}%`;
        suspRL.style.height = `${Math.min(100, (susp_rl / maxSusp) * 100)}%`;
        suspRR.style.height = `${Math.min(100, (susp_rr / maxSusp) * 100)}%`;

        // Pitch, Roll, Yaw (bytes 56, 60, 64)
        const pitch = view.getFloat32(56, true);
        const roll = view.getFloat32(60, true);
        const yaw = view.getFloat32(64, true);
        
        valPitch.textContent = pitch.toFixed(2);
        valRoll.textContent = roll.toFixed(2);
        valYaw.textContent = yaw.toFixed(2);

        // Motion Motor Actuator calculation
        const gain = parseFloat(inGain.value) || 1.0;
        let m1_current = (pitch + roll) * gain;
        let m2_current = (pitch - roll) * gain;
        let m3_current = sway * gain;

        if (!toggleAutoscale.checked) {
            m1_current = Math.max(-10, Math.min(10, m1_current));
            m2_current = Math.max(-10, Math.min(10, m2_current));
            m3_current = Math.max(-10, Math.min(10, m3_current));
        }

        histM1.push(m1_current);
        histM2.push(m2_current);
        histM3.push(m3_current);

        if (histM1.length > MAX_HISTORY) histM1.shift();
        if (histM2.length > MAX_HISTORY) histM2.shift();
        if (histM3.length > MAX_HISTORY) histM3.shift();

        valM1.textContent = m1_current.toFixed(2);
        valM2.textContent = m2_current.toFixed(2);
        valM3.textContent = m3_current.toFixed(2);

    } else {
        valSpeed.textContent = "ERR";
    }
});

// ----------------------------------------------------
// Futuristic Motor Simulation & Plotting (History & Autoscale)
// ----------------------------------------------------
const canvas = document.getElementById('motorPlot');
const ctx = canvas.getContext('2d');
const valM1 = document.getElementById('val-m1');
const valM2 = document.getElementById('val-m2');
const valM3 = document.getElementById('val-m3');
const inGain = document.getElementById('input-gain');
const toggleAutoscale = document.getElementById('toggle-autoscale');

function resizeCanvas() {
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = 250;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

const MAX_HISTORY = 100;
let histM1 = [];
let histM2 = [];
let histM3 = [];

// Base vertical scale if autoscale is off
const BASE_SCALE = 20; 

function drawPlot() {
    // Fill background with a slight fade to create a glowing trail effect for the moving lines
    ctx.fillStyle = 'rgba(5, 6, 8, 0.4)';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    const cy = canvas.height / 2;

    // Draw Center Line
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctx.beginPath();
    ctx.moveTo(0, cy);
    ctx.lineTo(canvas.width, cy);
    ctx.stroke();

    // Determine Scale
    let currentScale = BASE_SCALE;
    if (toggleAutoscale.checked) {
        let maxVal = 0.1; // minimum threshold to prevent divide by zero / infinite scale
        for (let i = 0; i < histM1.length; i++) {
            maxVal = Math.max(maxVal, Math.abs(histM1[i]), Math.abs(histM2[i]), Math.abs(histM3[i]));
        }
        // We want maxVal to map to 90% of the half-height
        currentScale = (cy * 0.9) / maxVal;
    }

    // Helper to draw a glowing history line with fast GPU alpha layering
    function drawHistoryLine(history, color) {
        if (history.length === 0) return;
        const stepX = canvas.width / MAX_HISTORY;
        const startX = canvas.width - (history.length * stepX);

        // Glow Outline Pass
        ctx.lineWidth = 4;
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.28;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        for (let i = 0; i < history.length; i++) {
            const x = startX + (i * stepX);
            const y = cy - (history[i] * currentScale);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // Sharp Core Line Pass
        ctx.lineWidth = 1.8;
        ctx.globalAlpha = 1.0;
        ctx.beginPath();
        for (let i = 0; i < history.length; i++) {
            const x = startX + (i * stepX);
            const y = cy - (history[i] * currentScale);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        
        // Draw head dot
        const lastVal = history[history.length - 1];
        const headY = cy - (lastVal * currentScale);

        ctx.globalAlpha = 0.4;
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(canvas.width, headY, 6, 0, Math.PI * 2);
        ctx.fill();

        ctx.globalAlpha = 1.0;
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(canvas.width, headY, 2.5, 0, Math.PI * 2);
        ctx.fill();
    }

    drawHistoryLine(histM1, '#ff3366');
    drawHistoryLine(histM2, '#00e5ff');
    drawHistoryLine(histM3, '#00e676');
    
    requestAnimationFrame(drawPlot);
}
drawPlot();
