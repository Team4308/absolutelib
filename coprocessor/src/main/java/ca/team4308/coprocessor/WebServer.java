package ca.team4308.coprocessor;

import io.javalin.Javalin;
import io.javalin.websocket.WsContext;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

public class WebServer {

    private final Javalin app;
    private final ObjectMapper mapper = new ObjectMapper();
    private final Set<WsContext> wsSessions = ConcurrentHashMap.newKeySet();

    public WebServer(TCPServer tcpServer, ca.team4308.absolutelib.network.task.server.TaskServer taskServer) {
        app = Javalin.create(config -> {
            config.staticFiles.add(staticFiles -> {
                staticFiles.hostedPath = "/";
                staticFiles.directory = "site";
                staticFiles.location = io.javalin.http.staticfiles.Location.EXTERNAL;
            });
        }).start(Config.HTTP_PORT);

        // WebSocket for realtime telemetry
        app.ws("/ws/telemetry", ws -> {
            ws.onConnect(ctx -> {
                wsSessions.add(ctx);
                System.out.println("WebSocket client connected from " + ctx.session.getRemoteAddress());
            });
            ws.onClose(ctx -> {
                wsSessions.remove(ctx);
                System.out.println("WebSocket client disconnected");
            });
            ws.onError(ctx -> {
                wsSessions.remove(ctx);
                System.err.println("WebSocket error: " + ctx.error().getMessage());
            });
        });

        // Background thread to push telemetry to WebSocket clients
        Thread wsPushThread = new Thread(() -> {
            long lastSeq = -1;
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    ca.team4308.absolutelib.math.trajectories.network.FullTelemetryPacket packet = tcpServer.latestFullTelemetry.get();
                    if (packet != null && packet.sequence != lastSeq) {
                        if (!wsSessions.isEmpty()) {
                            for (WsContext session : wsSessions) {
                                if (session.session.isOpen()) {
                                    session.send(mapper.writeValueAsString(packet));
                                }
                            }
                        }
                        lastSeq = packet.sequence;
                    }
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    break;
                } catch (Exception e) {
                    // Silently ignore or log once
                }
            }
        });
        wsPushThread.setDaemon(true);
        wsPushThread.start();

        app.get("/", ctx -> {
            ctx.html(
                    """
                            <!DOCTYPE html>
                            <html lang="en">
                            <head>
                                <meta charset="UTF-8">
                                <title>Trajectory Dashboard</title>
                                <link rel="preconnect" href="https://fonts.googleapis.com">
                                <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
                                <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;700&family=JetBrains+Mono:wght@400;700&display=swap" rel="stylesheet">
                                <style>
                                    :root {
                                        --bg: #0a0b0e;
                                        --card-bg: #14171d;
                                        --accent: #4fc3f7;
                                        --accent-glow: rgba(79, 195, 247, 0.3);
                                        --text: #e0e6ed;
                                        --text-dim: #94a3b8;
                                        --success: #10b981;
                                        --error: #ef4444;
                                        --warning: #f59e0b;
                                        --border: rgba(255,255,255,0.08);
                                    }
                                    * { box-sizing: border-box; }
                                    body { 
                                        font-family: 'Outfit', sans-serif; 
                                        background: var(--bg); 
                                        color: var(--text); 
                                        margin: 0; 
                                        padding: 20px;
                                        overflow-x: hidden;
                                    }
                                    .dashboard {
                                        display: grid;
                                        grid-template-columns: 350px 1fr 350px;
                                        grid-template-rows: auto 1fr auto;
                                        gap: 20px;
                                        max-width: 1800px;
                                        margin: 0 auto;
                                    }
                                    header {
                                        grid-column: 1 / span 3;
                                        display: flex;
                                        justify-content: space-between;
                                        align-items: center;
                                        padding: 0 10px;
                                        margin-bottom: 10px;
                                    }
                                    h1 { margin: 0; font-size: 1.8rem; font-weight: 700; letter-spacing: -0.5px; }
                                    .status-pill {
                                        display: flex;
                                        align-items: center;
                                        gap: 8px;
                                        background: var(--card-bg);
                                        padding: 6px 14px;
                                        border-radius: 20px;
                                        border: 1px solid var(--border);
                                        font-size: 0.9rem;
                                        font-weight: 600;
                                    }
                                    .indicator { width: 8px; height: 8px; border-radius: 50%; background: var(--error); box-shadow: 0 0 8px var(--error); transition: 0.3s; }
                                    .connected .indicator { background: var(--success); box-shadow: 0 0 10px var(--success); }
                                    
                                    .card {
                                        background: var(--card-bg);
                                        border: 1px solid var(--border);
                                        border-radius: 16px;
                                        padding: 20px;
                                        box-shadow: 0 4px 20px rgba(0,0,0,0.3);
                                        transition: transform 0.2s;
                                    }
                                    .card h2 { margin: 0 0 15px 0; font-size: 1.1rem; color: var(--accent); text-transform: uppercase; letter-spacing: 1px; }
                                    
                                    .viz-container {
                                        grid-column: 2;
                                        grid-row: 1 / span 2;
                                        display: flex;
                                        flex-direction: column;
                                        gap: 20px;
                                    }
                                    
                                    #flightPathCanvas {
                                        width: 100%;
                                        height: 500px;
                                        background: #0f1115;
                                        border-radius: 16px;
                                        border: 1px solid var(--border);
                                    }
                                    
                                    .stats-grid {
                                        display: grid;
                                        grid-template-columns: 1fr 1fr;
                                        gap: 12px;
                                    }
                                    .stat-item {
                                        display: flex;
                                        flex-direction: column;
                                        gap: 4px;
                                    }
                                    .stat-label { font-size: 0.75rem; color: var(--text-dim); text-transform: uppercase; }
                                    .stat-value { font-family: 'JetBrains Mono', monospace; font-size: 1.2rem; font-weight: 700; color: #fff; }
                                    .stat-unit { font-size: 0.8rem; font-weight: 400; color: var(--text-dim); margin-left: 2px; }

                                    .solve-stats { font-size: 0.9rem; color: var(--text-dim); }
                                    .rejection-item { display: flex; justify-content: space-between; margin-bottom: 4px; }
                                    .rej-val { color: var(--error); font-family: 'JetBrains Mono'; }
                                    .rej-acc { color: var(--success); }
                                    
                                    .footer {
                                        grid-column: 1 / span 3;
                                        display: flex;
                                        justify-content: space-between;
                                        padding: 10px;
                                        font-size: 0.8rem;
                                        color: var(--text-dim);
                                        border-top: 1px solid var(--border);
                                    }
                                    
                                    /* Progress bar for flywheel power */
                                    .power-bar-bg { width: 100%; height: 6px; background: rgba(255,255,255,0.05); border-radius: 3px; margin-top: 8px; overflow: hidden; }
                                    .power-bar-fill { height: 100%; background: var(--accent); box-shadow: 0 0 10px var(--accent-glow); transition: width 0.1s; }
                                    
                                    .disconnected-overlay {
                                        position: fixed; top: 0; left: 0; right: 0; bottom: 0;
                                        background: rgba(0,0,0,0.7);
                                        backdrop-filter: blur(4px);
                                        display: flex; justify-content: center; align-items: center;
                                        z-index: 1000;
                                        opacity: 0; pointer-events: none; transition: 0.3s;
                                    }
                                    .is-disconnected .disconnected-overlay { opacity: 1; pointer-events: all; }
                                </style>
                            </head>
                            <body>
                                <div class="disconnected-overlay">
                                    <div style="text-align: center;">
                                        <h1 style="color: var(--error); margin-bottom: 10px;">COMMUNICATION LOST</h1>
                                        <p>Reconnecting to Coprocessor WebSocket...</p>
                                    </div>
                                </div>

                                <div class="dashboard" id="dashboard">
                                    <header>
                                        <h1><span style="color: var(--accent); font-weight: 300;">TRAJECTORY</span></h1>
                                        <div class="status-pill" id="ws-status">
                                            <div class="indicator"></div>
                                            <span id="status-text">OFFLINE</span>
                                        </div>
                                    </header>

                                    <!-- Left Column -->
                                    <div style="display: flex; flex-direction: column; gap: 20px;">
                                        <div class="card">
                                            <h2>Shot Output</h2>
                                            <div class="stats-grid">
                                                <div class="stat-item"><span class="stat-label">Pitch</span><span class="stat-value"><span id="pitch">0.0</span><span class="stat-unit">°</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Yaw Adj</span><span class="stat-value"><span id="yaw">0.0</span><span class="stat-unit">°</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Target RPM</span><span class="stat-value"><span id="rpm">0</span><span class="stat-unit">RPM</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Exit Vel</span><span class="stat-value"><span id="exitVel">0.0</span><span class="stat-unit">m/s</span></span></div>
                                            </div>
                                            <div style="margin-top: 15px; display: flex; align-items: center; gap: 10px;">
                                                <div id="valid-badge" style="padding: 4px 10px; border-radius: 6px; font-size: 0.8rem; font-weight: 700;">INVALID</div>
                                                <span id="status-msg" style="font-size: 0.85rem; color: var(--text-dim);">No data</span>
                                            </div>
                                        </div>

                                        <div class="card">
                                            <h2>Flywheel Sim</h2>
                                            <div class="stats-grid">
                                                <div class="stat-item"><span class="stat-label">Efficiency</span><span class="stat-value"><span id="fwEff">0</span><span class="stat-unit">%</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Slip</span><span class="stat-value"><span id="fwSlip">0</span><span class="stat-unit">%</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Spin Up</span><span class="stat-value"><span id="fwSpinUp">0.0</span><span class="stat-unit">s</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Current</span><span class="stat-value"><span id="fwAmps">0.0</span><span class="stat-unit">A</span></span></div>
                                            </div>
                                            <div class="stat-item" style="margin-top: 15px;">
                                                <span class="stat-label">Motor Power: <span id="fwPowerTxt" style="color:#fff">0%</span></span>
                                                <div class="power-bar-bg"><div id="fwPowerBar" class="power-bar-fill" style="width: 0%"></div></div>
                                            </div>
                                            <div style="margin-top: 10px; font-size: 0.8rem; color: var(--text-dim);">
                                                Limiting: <span id="fwLimiting" style="color:#fff">None</span>
                                            </div>
                                        </div>

                                        <div class="card">
                                            <h2>Solver Trace</h2>
                                            <div class="rejection-item"><span>Mode</span><span id="trMode" style="color:var(--accent)">--</span></div>
                                            <div class="rejection-item"><span>Time</span><span class="stat-unit"><span id="trTime" class="rej-acc">0.0</span> ms</span></div>
                                            <div class="rejection-item"><span>Iterations</span><span id="trIter" class="rej-acc">0</span></div>
                                            <div style="margin: 10px 0; border-top: 1px solid var(--border);"></div>
                                            <div class="rejection-item"><span>Accepted</span><span id="trAcc" class="rej-acc">0</span></div>
                                            <div class="rejection-item"><span>Rej: Collision</span><span id="trRejCol" class="rej-val">0</span></div>
                                            <div class="rejection-item"><span>Rej: Low Arc</span><span id="trRejLow" class="rej-val">0</span></div>
                                            <div class="rejection-item"><span>Rej: Miss</span><span id="trRejMiss" class="rej-val">0</span></div>
                                            <div class="rejection-item"><span>Rej: Flyover</span><span id="trRejFly" class="rej-val">0</span></div>
                                        </div>
                                    </div>

                                    <!-- Middle Column -->
                                    <div class="viz-container">
                                        <canvas id="flightPathCanvas"></canvas>
                                        <div class="card" style="flex: 1;">
                                            <h2>Trajectory Metrics</h2>
                                            <div class="stats-grid">
                                                <div class="stat-item"><span class="stat-label">Confidence</span><span class="stat-value"><span id="metConf">0</span><span class="stat-unit">%</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Margin of Error</span><span class="stat-value"><span id="metMargin">0.0</span><span class="stat-unit">m</span></span></div>
                                                <div class="stat-item"><span class="stat-label">TOF</span><span class="stat-value"><span id="metTof">0.0</span><span class="stat-unit">s</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Max Height</span><span class="stat-value"><span id="metMaxH">0.0</span><span class="stat-unit">m</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Distance</span><span class="stat-value"><span id="metDist">0.0</span><span class="stat-unit">m</span></span></div>
                                                <div class="stat-item"><span class="stat-label">Height Diff</span><span class="stat-value"><span id="metHDiff">0.0</span><span class="stat-unit">m</span></span></div>
                                            </div>
                                        </div>
                                    </div>

                                    <!-- Right Column -->
                                    <div style="display: flex; flex-direction: column; gap: 20px;">
                                        <div class="card">
                                            <h2>Targeting Input</h2>
                                            <p class="stat-label">Robot Pose</p>
                                            <div class="stat-value" style="font-size: 1rem; margin-bottom: 10px;">
                                                <span id="inRX">0.00</span>, <span id="inRY">0.00</span>, <span id="inRZ">0.00</span>
                                            </div>
                                            <p class="stat-label">Target Position</p>
                                            <div class="stat-value" style="font-size: 1rem; margin-bottom: 10px;">
                                                <span id="inTX">0.00</span>, <span id="inTY">0.00</span>, <span id="inTZ">0.00</span>
                                            </div>
                                            <p class="stat-label">Chassis Velocity</p>
                                            <div class="stat-value" style="font-size: 1rem;">
                                                Vx: <span id="inVX">0.0</span>, Vy: <span id="inVY">0.0</span>
                                            </div>
                                        </div>

                                        <div class="card" id="discrete-card">
                                            <h2>Discrete Solution (CRT)</h2>
                                            <div id="ds-none" style="color: var(--text-dim); font-style: italic;">No CRT solution found</div>
                                            <div id="ds-content" style="display: none;">
                                                <div class="stats-grid">
                                                    <div class="stat-item"><span class="stat-label">RPM</span><span class="stat-value" id="dsRpm">0</span></div>
                                                    <div class="stat-item"><span class="stat-label">Angle</span><span class="stat-value"><span id="dsPitch">0.0</span><span class="stat-unit">°</span></span></div>
                                                    <div class="stat-item"><span class="stat-label">RPM Ticks</span><span class="stat-value" id="dsRpmTicks">0</span></div>
                                                    <div class="stat-item"><span class="stat-label">Angle Ticks</span><span class="stat-value" id="dsAngleTicks">0</span></div>
                                                </div>
                                                <div style="margin-top: 10px; font-weight: 600; color: var(--success);">
                                                    Score: <span id="dsScore">0.00</span>
                                                </div>
                                            </div>
                                        </div>

                                        <div class="card">
                                            <h2>Network Metrics</h2>
                                            <div class="rejection-item"><span>Sequence</span><span id="seq" style="color:var(--accent); font-family: 'JetBrains Mono';">0</span></div>
                                            <div class="rejection-item"><span>Incoming</span><span id="netIn">0</span></div>
                                            <div class="rejection-item"><span>Outgoing</span><span id="netOut">0</span></div>
                                            <div class="rejection-item"><span>TCP Latency</span><span id="netLatency">0ms</span></div>
                                        </div>
                                    </div>

                                    <div class="footer">
                                        <span>ABSOLUTELIB V2 COPROCESSOR</span>
                                        <span id="timestamp">--</span>
                                        <span>TEAM 4308</span>
                                    </div>
                                </div>

                                <script>
                                    const canvas = document.getElementById('flightPathCanvas');
                                    const ctx = canvas.getContext('2d');
                                    let ws;
                                    let lastData = null;
                                    let drawQueued = false;
                                    let canvasCssWidth = 0;
                                    let canvasCssHeight = 0;
                                    let canvasDpr = 0;
                                    let targetPath = [];
                                    let renderedPath = [];
                                    let targetActivePath = [];
                                    let renderedActivePath = [];
                                    let renderedFrame = null;
                                    let lastDrawTimeMs = 0;
                                    const pathSmoothingTauSec = 0.045;

                                    function connect() {
                                        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
                                        ws = new WebSocket(`${protocol}//${window.location.host}/ws/telemetry`);
                                        
                                        ws.onopen = () => {
                                            document.getElementById('ws-status').classList.add('connected');
                                            document.getElementById('status-text').innerText = 'ONLINE';
                                            document.body.classList.remove('is-disconnected');
                                        };
                                        
                                        ws.onclose = () => {
                                            document.getElementById('ws-status').classList.remove('connected');
                                            document.getElementById('status-text').innerText = 'OFFLINE';
                                            document.body.classList.add('is-disconnected');
                                            setTimeout(connect, 1000);
                                        };
                                        
                                        ws.onmessage = (event) => {
                                            const data = JSON.parse(event.data);
                                            updateUI(data);
                                            lastData = data;
                                            targetPath = extractPath(data);
                                            targetActivePath = extractActivePath(data);
                                            if (renderedPath.length !== targetPath.length) {
                                                renderedPath = targetPath.map(p => ({ ...p }));
                                            }
                                            if (renderedActivePath.length !== targetActivePath.length) {
                                                renderedActivePath = targetActivePath.map(p => ({ ...p }));
                                            }
                                            if (!renderedFrame) {
                                                renderedFrame = extractFrame(data);
                                            }
                                            scheduleDraw();
                                        };
                                    }

                                    function updateUI(data) {
                                        document.getElementById('seq').innerText = data.sequence;
                                        document.getElementById('timestamp').innerText = new Date(data.timestamp * 1000).toLocaleTimeString();
                                        
                                        // Output
                                        document.getElementById('pitch').innerText = data.pitch_deg.toFixed(2);
                                        document.getElementById('yaw').innerText = data.yaw_deg.toFixed(2);
                                        document.getElementById('rpm').innerText = data.rpm.toFixed(0);
                                        document.getElementById('exitVel').innerText = data.exit_vel_mps.toFixed(2);
                                        
                                        const badge = document.getElementById('valid-badge');
                                        badge.innerText = data.status;
                                        badge.style.background = data.status === 'SUCCESS' ? 'var(--success)' : 
                                                                data.status === 'MARGINAL' ? 'var(--warning)' : 'var(--error)';
                                        document.getElementById('status-msg').innerText = data.status === 'SUCCESS' ? 'Optimal Solution' : 'Check constraints';

                                        // Flywheel
                                        document.getElementById('fwEff').innerText = (data.fw_efficiency * 100).toFixed(0);
                                        document.getElementById('fwSlip').innerText = (data.fw_slip_ratio * 100).toFixed(1);
                                        document.getElementById('fwSpinUp').innerText = data.fw_spin_up_sec.toFixed(2);
                                        document.getElementById('fwAmps').innerText = data.fw_current_amps.toFixed(1);
                                        document.getElementById('fwPowerTxt').innerText = (data.fw_motor_power * 100).toFixed(0) + '%';
                                        document.getElementById('fwPowerBar').style.width = (data.fw_motor_power * 100) + '%';
                                        document.getElementById('fwLimiting').innerText = data.fw_limiting_factor;

                                        // Solver
                                        document.getElementById('trMode').innerText = data.tr_mode;
                                        document.getElementById('trTime').innerText = data.tr_time_ms.toFixed(2);
                                        document.getElementById('trIter').innerText = data.tr_iter;
                                        document.getElementById('trAcc').innerText = data.tr_accepted;
                                        document.getElementById('trRejCol').innerText = data.tr_rej_col;
                                        document.getElementById('trRejLow').innerText = data.tr_rej_low;
                                        document.getElementById('trRejMiss').innerText = data.tr_rej_miss;
                                        document.getElementById('trRejFly').innerText = data.tr_rej_fly;

                                        // Metrics
                                        document.getElementById('metConf').innerText = (data.confidence * 100).toFixed(0);
                                        document.getElementById('metMargin').innerText = data.met_margin_error.toFixed(3);
                                        document.getElementById('metTof').innerText = data.met_tof.toFixed(2);
                                        document.getElementById('metMaxH').innerText = data.met_max_height.toFixed(2);
                                        document.getElementById('metDist').innerText = data.met_dist.toFixed(2);
                                        document.getElementById('metHDiff').innerText = data.met_height_diff.toFixed(2);

                                        // Input
                                        document.getElementById('inRX').innerText = data.in_rx.toFixed(2);
                                        document.getElementById('inRY').innerText = data.in_ry.toFixed(2);
                                        document.getElementById('inRZ').innerText = data.in_rz.toFixed(2);
                                        document.getElementById('inTX').innerText = data.in_tx.toFixed(2);
                                        document.getElementById('inTY').innerText = data.in_ty.toFixed(2);
                                        document.getElementById('inTZ').innerText = data.in_tz.toFixed(2);
                                        document.getElementById('inVX').innerText = data.in_vx.toFixed(1);
                                        document.getElementById('inVY').innerText = data.in_vy.toFixed(1);

                                        // Discrete
                                        if (data.ds_valid) {
                                            document.getElementById('ds-none').style.display = 'none';
                                            document.getElementById('ds-content').style.display = 'block';
                                            document.getElementById('dsRpm').innerText = data.ds_rpm.toFixed(0);
                                            document.getElementById('dsPitch').innerText = data.ds_pitch_deg.toFixed(1);
                                            document.getElementById('dsRpmTicks').innerText = data.ds_rpm_ticks;
                                            document.getElementById('dsAngleTicks').innerText = data.ds_angle_ticks;
                                            document.getElementById('dsScore').innerText = data.ds_score.toFixed(2);
                                        } else {
                                            document.getElementById('ds-none').style.display = 'block';
                                            document.getElementById('ds-content').style.display = 'none';
                                        }
                                    }

                                    function scheduleDraw() {
                                        if (drawQueued) return;
                                        drawQueued = true;
                                        requestAnimationFrame((nowMs) => {
                                            drawQueued = false;
                                            draw(nowMs);
                                            if (lastData) scheduleDraw();
                                        });
                                    }

                                    function extractPath(data) {
                                        const count = data.path_count || 0;
                                        const path = [];
                                        for (let i = 0; i < count; i++) {
                                            path.push({
                                                x: data.flight_path_x[i],
                                                y: data.flight_path_y[i],
                                                z: data.flight_path_z[i],
                                            });
                                        }
                                        return path;
                                    }

                                    function extractActivePath(data) {
                                        const count = data.active_path_count || 0;
                                        const path = [];
                                        for (let i = 0; i < count; i++) {
                                            path.push({
                                                x: data.active_path_x[i],
                                                y: data.active_path_y[i],
                                                z: data.active_path_z[i],
                                            });
                                        }
                                        return path;
                                    }

                                    function extractFrame(data) {
                                        return {
                                            robotX: data.in_rx,
                                            robotY: data.in_ry,
                                            robotZ: data.in_rz,
                                            targetX: data.in_tx,
                                            targetY: data.in_ty,
                                            targetZ: data.in_tz,
                                            dist: data.met_dist,
                                            maxHeight: data.met_max_height,
                                        };
                                    }

                                    function lerp(a, b, t) {
                                        return a + (b - a) * t;
                                    }

                                    function smoothVisuals(nowMs) {
                                        if (!lastData) return null;

                                        const dtSec = lastDrawTimeMs > 0 ? Math.min(0.05, (nowMs - lastDrawTimeMs) / 1000.0) : 0.016;
                                        lastDrawTimeMs = nowMs;
                                        const alpha = 1.0 - Math.exp(-dtSec / pathSmoothingTauSec);
                                        const targetFrame = extractFrame(lastData);

                                        if (!renderedFrame) {
                                            renderedFrame = targetFrame;
                                        } else {
                                            renderedFrame.robotX = lerp(renderedFrame.robotX, targetFrame.robotX, alpha);
                                            renderedFrame.robotY = lerp(renderedFrame.robotY, targetFrame.robotY, alpha);
                                            renderedFrame.robotZ = lerp(renderedFrame.robotZ, targetFrame.robotZ, alpha);
                                            renderedFrame.targetX = lerp(renderedFrame.targetX, targetFrame.targetX, alpha);
                                            renderedFrame.targetY = lerp(renderedFrame.targetY, targetFrame.targetY, alpha);
                                            renderedFrame.targetZ = lerp(renderedFrame.targetZ, targetFrame.targetZ, alpha);
                                            renderedFrame.dist = lerp(renderedFrame.dist, targetFrame.dist, alpha);
                                            renderedFrame.maxHeight = lerp(renderedFrame.maxHeight, targetFrame.maxHeight, alpha);
                                        }

                                        if (targetPath.length === 0) {
                                            renderedPath = [];
                                        } else if (renderedPath.length !== targetPath.length) {
                                            renderedPath = targetPath.map(p => ({ ...p }));
                                        } else {
                                            for (let i = 0; i < targetPath.length; i++) {
                                                renderedPath[i].x = lerp(renderedPath[i].x, targetPath[i].x, alpha);
                                                renderedPath[i].y = lerp(renderedPath[i].y, targetPath[i].y, alpha);
                                                renderedPath[i].z = lerp(renderedPath[i].z, targetPath[i].z, alpha);
                                            }
                                        }

                                        if (targetActivePath.length === 0) {
                                            renderedActivePath = [];
                                        } else if (renderedActivePath.length !== targetActivePath.length) {
                                            renderedActivePath = targetActivePath.map(p => ({ ...p }));
                                        } else {
                                            for (let i = 0; i < targetActivePath.length; i++) {
                                                renderedActivePath[i].x = lerp(renderedActivePath[i].x, targetActivePath[i].x, alpha);
                                                renderedActivePath[i].y = lerp(renderedActivePath[i].y, targetActivePath[i].y, alpha);
                                                renderedActivePath[i].z = lerp(renderedActivePath[i].z, targetActivePath[i].z, alpha);
                                            }
                                        }

                                        return renderedFrame;
                                    }

                                    function resizeCanvasIfNeeded() {
                                        const dpr = window.devicePixelRatio || 1;
                                        const displayW = canvas.clientWidth;
                                        const displayH = canvas.clientHeight;
                                        if (displayW !== canvasCssWidth || displayH !== canvasCssHeight || dpr !== canvasDpr) {
                                            canvasCssWidth = displayW;
                                            canvasCssHeight = displayH;
                                            canvasDpr = dpr;
                                            canvas.width = Math.max(1, Math.floor(displayW * dpr));
                                            canvas.height = Math.max(1, Math.floor(displayH * dpr));
                                            ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
                                        }
                                        return { displayW, displayH };
                                    }

                                    function draw(nowMs) {
                                        if (!lastData) return;
                                        
                                        const { displayW, displayH } = resizeCanvasIfNeeded();
                                        const frame = smoothVisuals(nowMs);
                                        if (!frame) return;
                                        
                                        ctx.clearRect(0, 0, displayW, displayH);
                                        
                                        // Draw Grid
                                        ctx.strokeStyle = 'rgba(255,255,255,0.05)';
                                        ctx.lineWidth = 1;
                                        for(let x=0; x<displayW; x+=50) { ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,displayH); ctx.stroke(); }
                                        for(let y=0; y<displayH; y+=50) { ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(displayW,y); ctx.stroke(); }

                                        if (renderedPath.length === 0 && renderedActivePath.length === 0) return;

                                        // Calculate Bounds
                                        const targetDist = frame.dist;
                                        const maxH = Math.max(frame.maxHeight, frame.targetZ, frame.robotZ) + 0.5;
                                        const maxX = targetDist + 1.0;
                                        
                                        const scaleX = (displayW - 100) / maxX;
                                        const scaleY = (displayH - 100) / maxH;
                                        const scale = Math.min(scaleX, scaleY);
                                        
                                        const offsetX = 50;
                                        const offsetY = displayH - 50;

                                        function toScreen(x, y) {
                                            return {
                                                x: offsetX + x * scale,
                                                y: offsetY - y * scale
                                            };
                                        }

                                        // Draw Hub
                                        const hub = toScreen(targetDist, frame.targetZ);
                                        ctx.fillStyle = 'rgba(20, 184, 129, 0.2)';
                                        ctx.strokeStyle = '#10b981';
                                        ctx.lineWidth = 2;
                                        ctx.strokeRect(hub.x - 15, hub.y, 30, 100);
                                        ctx.fillRect(hub.x - 15, hub.y, 30, 100);
                                        
                                        // Draw Robot
                                        const bot = toScreen(0, frame.robotZ);
                                        ctx.fillStyle = 'rgba(79, 195, 247, 0.4)';
                                        ctx.strokeStyle = '#4fc3f7';
                                        ctx.lineWidth = 2;
                                        ctx.strokeRect(bot.x - 10, bot.y - 10, 20, 20);
                                        ctx.fillRect(bot.x - 10, bot.y - 10, 20, 20);

                                        if (renderedActivePath.length > 0) {
                                            drawPath(renderedActivePath, frame, toScreen, '#f59e0b', 3, [8, 8]);
                                            ctx.fillStyle = '#f59e0b';
                                            ctx.font = '12px JetBrains Mono, monospace';
                                            ctx.fillText(`ACTIVE ${lastData.active_pitch_deg.toFixed(1)}deg ${lastData.active_rpm.toFixed(0)}RPM`, 18, 28);
                                        }

                                        // Draw Path
                                        if (renderedPath.length > 0) {
                                            drawPath(renderedPath, frame, toScreen, '#4fc3f7', 4, []);
                                        }
                                    }

                                    function drawPath(path, frame, toScreen, color, width, dash) {
                                        ctx.beginPath();
                                        ctx.lineWidth = width;
                                        ctx.lineCap = 'round';
                                        ctx.lineJoin = 'round';
                                        ctx.setLineDash(dash);
                                        ctx.strokeStyle = color; 

                                        // We need to calculate distance from robot for each point
                                        // The points are in global (x,y,z). 
                                        // Distance = sqrt((px-rx)^2 + (py-ry)^2)
                                        for (let i = 0; i < path.length; i++) {
                                            const dx = path[i].x - frame.robotX;
                                            const dy = path[i].y - frame.robotY;
                                            const dist = Math.sqrt(dx*dx + dy*dy);
                                            const z = path[i].z;
                                            
                                            const p = toScreen(dist, z);
                                            if (i === 0) ctx.moveTo(p.x, p.y);
                                            else ctx.lineTo(p.x, p.y);
                                        }
                                        ctx.stroke();
                                        ctx.setLineDash([]);
                                    }

                                    window.addEventListener('resize', () => {
                                        canvasCssWidth = 0;
                                        scheduleDraw();
                                    });
                                    connect();
                                </script>
                            </body>
                            </html>
                            """);
        });

        app.get("/api/status", ctx -> {
            Map<String, Object> status = new HashMap<>();
            status.put("isConnected", tcpServer.isConnected.get() || taskServer.isConnected.get());
            status.put("solverTimeMs", tcpServer.lastSolverTimeMs.get());
            status.put("latencyMs", tcpServer.lastSolverTimeMs.get());
            status.put("totalRequests", tcpServer.totalRequests.get());
            status.put("droppedPackets", tcpServer.droppedPackets.get());
            status.put("request", tcpServer.latestRequest.get());
            status.put("response", tcpServer.latestResponse.get());
            status.put("incomingPackets", tcpServer.incomingPackets.get() + taskServer.incomingPackets.get());
            status.put("outgoingPackets", tcpServer.outgoingPackets.get() + taskServer.outgoingPackets.get());
            status.put("battery", tcpServer.batteryLevel.get());

            ca.team4308.absolutelib.math.trajectories.network.FullTelemetryPacket full = tcpServer.latestFullTelemetry.get();
            status.put("fullTelemetry", full);
            
            ctx.json(status);
        });

        System.out.println("Web Server running on port " + Config.HTTP_PORT);
    }
}
