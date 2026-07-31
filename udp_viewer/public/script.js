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

// Helper to read float32 from byte array
function readFloatLE(data, offset) {
    if (offset + 4 > data.length) return 0;
    const buffer = new ArrayBuffer(4);
    const view = new DataView(buffer);
    data.slice(offset, offset + 4).forEach((b, i) => view.setUint8(i, b));
    return view.getFloat32(0, true);
}

// Telemetry Event
socket.on('telemetry', (packet) => {
    const data = packet.data; // Array of bytes
    
    // Update RAW view if active
    if (isRawView) {
        rawStats.textContent = `${packet.size} bytes from ${packet.address}:${packet.port}`;
        // Hex dump
        let hex = '';
        for (let i = 0; i < data.length; i++) {
            hex += data[i].toString(16).padStart(2, '0') + ' ';
            if ((i + 1) % 16 === 0) hex += '\n';
        }
        rawOutput.textContent = hex;
        return;
    }

    // Parse Extradata=3 (Common Codemasters Format)
    // 264 bytes total usually, containing 66 32-bit floats
    if (data.length >= 264) {
        
        // Speed (usually offset 7, which is byte 28)
        const speedMS = readFloatLE(data, 28);
        const speedKMH = Math.max(0, speedMS * 3.6);
        valSpeed.textContent = speedKMH.toFixed(0);

        // Gear (offset 33, byte 132) -> float representing gear (0 = R, 1 = N, 2 = 1, etc.)
        const gearFloat = readFloatLE(data, 132);
        let gearStr = "N";
        if (gearFloat === 0) gearStr = "R";
        else if (gearFloat > 1) gearStr = Math.round(gearFloat - 1).toString();
        valGear.textContent = gearStr;

        // RPM (offset 37, byte 148 * 10 is typical in DR2)
        const rpm = readFloatLE(data, 148) * 10;
        valRpm.textContent = rpm.toFixed(0);
        let rpmPct = Math.min(100, Math.max(0, (rpm / MAX_RPM) * 100));
        barRpm.style.width = `${rpmPct}%`;

        // G-Forces / Accel (Offsets vary by game, DR2 uses 30,31,32 or 39,40,41)
        // Let's read Acceleration X, Y, Z (offsets 30, 31, 32 -> bytes 120, 124, 128)
        const sway = readFloatLE(data, 120);
        const heave = readFloatLE(data, 124);
        const surge = readFloatLE(data, 128);
        
        valSway.textContent = sway.toFixed(2);
        valHeave.textContent = heave.toFixed(2);
        valSurge.textContent = surge.toFixed(2);

        // Update G-Dot
        // Map +/- 2G to +/- 50%
        const maxG = 2.0;
        let dotX = (sway / maxG) * 50 + 50;
        let dotY = (surge / maxG) * 50 + 50;
        // Clamp
        dotX = Math.max(0, Math.min(100, dotX));
        dotY = Math.max(0, Math.min(100, dotY));
        gDot.style.left = `${dotX}%`;
        gDot.style.top = `${dotY}%`;

        // Suspension Position (offsets 17, 18, 19, 20 -> bytes 68, 72, 76, 80)
        const susp_rl = readFloatLE(data, 68);
        const susp_rr = readFloatLE(data, 72);
        const susp_fl = readFloatLE(data, 76);
        const susp_fr = readFloatLE(data, 80);

        valSuspFL.textContent = susp_fl.toFixed(2);
        valSuspFR.textContent = susp_fr.toFixed(2);
        valSuspRL.textContent = susp_rl.toFixed(2);
        valSuspRR.textContent = susp_rr.toFixed(2);

        // Scale 0-100% (assuming max travel ~ 0.5m)
        const maxSusp = 0.5;
        suspFL.style.height = `${Math.min(100, (susp_fl / maxSusp) * 100)}%`;
        suspFR.style.height = `${Math.min(100, (susp_fr / maxSusp) * 100)}%`;
        suspRL.style.height = `${Math.min(100, (susp_rl / maxSusp) * 100)}%`;
        suspRR.style.height = `${Math.min(100, (susp_rr / maxSusp) * 100)}%`;

        // Pitch, Roll, Yaw (offsets 14, 15, 16 -> bytes 56, 60, 64) OR Rotation
        // Depending on DR2 specifics, we'll try to read typical offsets
        const pitch = readFloatLE(data, 56);
        const roll = readFloatLE(data, 60);
        const yaw = readFloatLE(data, 64);
        
        valPitch.textContent = pitch.toFixed(2);
        valRoll.textContent = roll.toFixed(2);
        valYaw.textContent = yaw.toFixed(2);

    } else {
        // Less than 264 bytes, probably extradata=0 or wrong format
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

    // Helper to draw a glowing history line
    function drawHistoryLine(history, color) {
        if (history.length === 0) return;
        
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.shadowBlur = 10;
        ctx.shadowColor = color;
        ctx.lineJoin = 'round';
        ctx.beginPath();

        const stepX = canvas.width / MAX_HISTORY;
        for (let i = 0; i < history.length; i++) {
            // Right to left scrolling (newest on right)
            const x = (canvas.width - (history.length * stepX)) + (i * stepX);
            const y = cy - (history[i] * currentScale);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.shadowBlur = 0; // reset
        
        // Draw bright dot at the head
        const lastVal = history[history.length - 1];
        ctx.fillStyle = '#fff';
        ctx.shadowBlur = 15;
        ctx.shadowColor = color;
        ctx.beginPath();
        ctx.arc(canvas.width, cy - (lastVal * currentScale), 4, 0, Math.PI*2);
        ctx.fill();
        ctx.shadowBlur = 0;
    }

    drawHistoryLine(histM1, '#ff3366');
    drawHistoryLine(histM2, '#00e5ff');
    drawHistoryLine(histM3, '#00e676');
    
    requestAnimationFrame(drawPlot);
}
drawPlot();

// Hook into telemetry to calculate motor values and push to history
socket.on('telemetry', (packet) => {
    if (packet.data.length < 264) return;
    const data = packet.data;
    
    const pitch = readFloatLE(data, 56);
    const roll = readFloatLE(data, 60);
    // Use Sway (accel X) for the 3rd DOF
    const sway = readFloatLE(data, 120); 
    
    const gain = parseFloat(inGain.value) || 1.0;

    // Simulate 2DOF (Pitch + Roll mix) and 1DOF (Sway)
    let m1_current = (pitch + roll) * gain;
    let m2_current = (pitch - roll) * gain;
    let m3_current = sway * gain;

    // Limit if autoscale is off, otherwise let it go wild
    if (!toggleAutoscale.checked) {
        m1_current = Math.max(-10, Math.min(10, m1_current));
        m2_current = Math.max(-10, Math.min(10, m2_current));
        m3_current = Math.max(-10, Math.min(10, m3_current));
    }

    // Push to history
    histM1.push(m1_current);
    histM2.push(m2_current);
    histM3.push(m3_current);

    if (histM1.length > MAX_HISTORY) histM1.shift();
    if (histM2.length > MAX_HISTORY) histM2.shift();
    if (histM3.length > MAX_HISTORY) histM3.shift();

    // Update Text
    valM1.textContent = m1_current.toFixed(2);
    valM2.textContent = m2_current.toFixed(2);
    valM3.textContent = m3_current.toFixed(2);
});
