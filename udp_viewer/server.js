const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const dgram = require('dgram');
const path = require('path');

const WEB_PORT = 3103;
const UDP_PORT = 20777;

// Setup Express and HTTP Server
const app = express();
const server = http.createServer(app);
const io = new Server(server);

// Serve static files from 'public' directory
app.use(express.static(path.join(__dirname, 'public')));

// Setup UDP Socket
const udpServer = dgram.createSocket('udp4');

udpServer.on('error', (err) => {
    console.error(`UDP Server error:\n${err.stack}`);
    udpServer.close();
});

udpServer.on('message', (msg, rinfo) => {
    // msg is a Buffer. Convert to array to send via JSON over WebSockets
    // We send raw bytes so the frontend can parse or display RAW as requested.
    io.emit('telemetry', {
        address: rinfo.address,
        port: rinfo.port,
        size: msg.length,
        timestamp: Date.now(),
        data: Array.from(msg)
    });
});

udpServer.on('listening', () => {
    const address = udpServer.address();
    console.log(`📡 UDP Server listening on port ${address.port}`);
});

udpServer.bind(UDP_PORT);

// Socket.io connection handling
io.on('connection', (socket) => {
    console.log('Client connected to Web UI');
    socket.on('disconnect', () => {
        console.log('Client disconnected from Web UI');
    });
});

// Start Web Server
server.listen(WEB_PORT, () => {
    console.log(`🚀 Web Dashboard running at http://localhost:${WEB_PORT}`);
});
