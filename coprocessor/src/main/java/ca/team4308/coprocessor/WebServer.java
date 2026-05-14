package ca.team4308.coprocessor;

import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

import java.util.HashMap;
import java.util.Map;

public class WebServer {

    private final Javalin app;

    public WebServer(TCPServer tcpServer, ca.team4308.absolutelib.network.task.server.TaskServer taskServer) {
        app = Javalin.create(config -> {
            // No static files needed, just raw handlers
        }).start(Config.HTTP_PORT);

        app.get("/", ctx -> {
            ctx.html(
                    """
                            <!DOCTYPE html>
                            <html lang="en">
                            <head>
                                <meta charset="UTF-8">
                                <title>Trajectory Coprocessor</title>
                                <style>
                                    body { font-family: sans-serif; padding: 20px; background: #121212; color: #e0e0e0; }
                                    .card { background: #1e1e1e; padding: 15px; border-radius: 8px; margin-bottom: 20px; }
                                    h2 { margin-top: 0; color: #4fc3f7; }
                                    table { width: 100%; border-collapse: collapse; }
                                    th, td { text-align: left; padding: 8px; border-bottom: 1px solid #333; }
                                    .disconnected .card { opacity: 0.35; }
                                    .status-connected { color: #5dfc5d; }
                                    .status-disconnected { color: #fc5d5d; }
                                    .small-label { font-size: 0.85rem; color: #bbbbbb; }
                                </style>
                                <script>
                                    async function fetchStatus() {
                                        const res = await fetch('/api/status');
                                        const data = await res.json();
                                        document.getElementById('connected').innerText = data.isConnected ? "CONNECTED" : "DISCONNECTED";
                                        document.getElementById('latency').innerText = data.latencyMs + " ms";

                                        document.body.classList.toggle('disconnected', !data.isConnected);
                                        document.getElementById('connected').className = data.isConnected ? 'status-connected' : 'status-disconnected';

                                        if (!data.isConnected) {
                                            // Don't show stale measurement values when disconnected
                                            document.getElementById('robotX').innerText = '--';
                                            document.getElementById('robotY').innerText = '--';
                                            document.getElementById('robotZ').innerText = '--';
                                            document.getElementById('pitch').innerText = '--';
                                            document.getElementById('yaw').innerText = '--';
                                            document.getElementById('rpm').innerText = '--';
                                            document.getElementById('valid').innerText = '--';
                                            document.getElementById('batteryPercentage').innerText = '--';
                                            document.getElementById('incoming').innerText = '--';
                                            document.getElementById('outgoing').innerText = '--';
                                            document.getElementById('lossyCount').innerText = '--';
                                            document.getElementById('lossyLast').innerText = '--';
                                            return;
                                        }

                                        document.getElementById('batteryPercentage').innerText = data.battery.toFixed(1) + '%';
                                        document.getElementById('incoming').innerText = data.incomingPackets;
                                        document.getElementById('outgoing').innerText = data.outgoingPackets;

                                        if (data.request) {
                                            document.getElementById('robotX').innerText = data.request.robot_x.toFixed(2);
                                            document.getElementById('robotY').innerText = data.request.robot_y.toFixed(2);
                                            document.getElementById('robotZ').innerText = (data.request.robot_z || 0.0).toFixed(2);
                                        }
                                        if (data.response) {
                                            document.getElementById('pitch').innerText = data.response.pitch_deg.toFixed(2);
                                            document.getElementById('yaw').innerText = data.response.yaw_deg.toFixed(2);
                                            document.getElementById('rpm').innerText = data.response.rpm.toFixed(0);
                                            document.getElementById('valid').innerText = data.response.valid;
                                        }

                                        if (data.lossy && data.lossy.count !== undefined) {
                                            document.getElementById('lossyCount').innerText = data.lossy.count;
                                            if (data.lossy.last) {
                                                const lp = data.lossy.last;
                                                document.getElementById('lossyLast').innerText =
                                                    `${lp.x.toFixed(2)}, ${lp.y.toFixed(2)}, ${lp.z.toFixed(2)}`;
                                            } else {
                                                document.getElementById('lossyLast').innerText = '--';
                                            }
                                        }

                                        if (data.lossy && Array.isArray(data.lossy.points)) {
                                            renderLossySvg(data.lossy.points);
                                        }
                                    }
                                    setInterval(fetchStatus, 100);

                                    function renderLossySvg(points) {
                                        const svg = document.getElementById('lossySvg');
                                        const poly = document.getElementById('lossyPolyline');
                                        if (!svg || !poly || points.length === 0) {
                                            if (poly) {
                                                poly.setAttribute('points', '');
                                            }
                                            return;
                                        }

                                        let minX = points[0].x;
                                        let maxX = points[0].x;
                                        let minY = points[0].y;
                                        let maxY = points[0].y;

                                        for (const p of points) {
                                            minX = Math.min(minX, p.x);
                                            maxX = Math.max(maxX, p.x);
                                            minY = Math.min(minY, p.y);
                                            maxY = Math.max(maxY, p.y);
                                        }

                                        const width = 360;
                                        const height = 160;
                                        const padding = 10;
                                        const spanX = Math.max(0.001, maxX - minX);
                                        const spanY = Math.max(0.001, maxY - minY);

                                        const pts = points.map(p => {
                                            const x = padding + ((p.x - minX) / spanX) * (width - padding * 2);
                                            const y = padding + (1.0 - (p.y - minY) / spanY) * (height - padding * 2);
                                            return `${x.toFixed(1)},${y.toFixed(1)}`;
                                        }).join(' ');

                                        poly.setAttribute('points', pts);
                                    }
                                </script>
                            </head>
                            <body>
                                <h1>Trajectory Coprocessor Dashboard</h1>

                                <div class="card" id="status-container">
                                    <h2>Status</h2>
                                    <p><strong>Connection:</strong> <span id="connected">UNKNOWN</span></p>
                                    <p><strong>TCP Latency:</strong> <span id="latency">0 ms</span></p>
                                    <p><strong>Version Hash:</strong> <span>v1.0.0</span></p>
                                </div>

                                <div class="card">
                                    <h2>Latest Input</h2>
                                    <table>
                                        <tr><th>Robot X (m)</th><td id="robotX">0.0</td></tr>
                                        <tr><th>Robot Y (m)</th><td id="robotY">0.0</td></tr>
                                        <tr><th>Robot Z (m)</th><td id="robotZ">0.0</td></tr>
                                    </table>
                                </div>

                                <div class="card">
                                    <h2>Latest Output</h2>
                                    <table>
                                        <tr><th>Pitch (deg)</th><td id="pitch">0.0</td></tr>
                                        <tr><th>Yaw (deg)</th><td id="yaw">0.0</td></tr>
                                        <tr><th>RPM</th><td id="rpm">0</td></tr>
                                        <tr><th>Valid</th><td id="valid">false</td></tr>
                                    </table>
                                </div>

                                <div class="card">
                                    <h2>Hardware info</h2>
                                    <table>
                                        <tr><th>Battery Percentage</th><td id="batteryPercentage">0</td></tr>
                                        <tr><th>Incoming Packets</th><td id="incoming">0</td></tr>
                                        <tr><th>Outgoing Packets</th><td id="outgoing">0</td></tr>
                                    </table>
                                </div>

                                <div class="card">
                                    <h2>Lossy Flight Path</h2>
                                    <table>
                                        <tr><th>Points Received</th><td id="lossyCount">0</td></tr>
                                        <tr><th>Last Point (x, y, z)</th><td id="lossyLast">--</td></tr>
                                    </table>
                                    <svg id="lossySvg" width="360" height="160" style="margin-top: 10px; background: #141414; border: 1px solid #333; border-radius: 6px;">
                                        <polyline id="lossyPolyline" fill="none" stroke="#4fc3f7" stroke-width="2" points="" />
                                    </svg>
                                </div>
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

            Map<String, Object> lossy = new HashMap<>();
            ca.team4308.absolutelib.math.trajectories.network.LossyDataPacket lossyPacket = tcpServer.latestLossyPacket.get();
            if (lossyPacket != null && lossyPacket.flightPath != null) {
                lossy.put("count", lossyPacket.flightPath.size());
                if (!lossyPacket.flightPath.isEmpty()) {
                    ca.team4308.absolutelib.math.trajectories.impl.Pose3d last =
                            lossyPacket.flightPath.get(lossyPacket.flightPath.size() - 1);
                    Map<String, Object> lastPoint = new HashMap<>();
                    lastPoint.put("x", last.getTranslation().x);
                    lastPoint.put("y", last.getTranslation().y);
                    lastPoint.put("z", last.getTranslation().z);
                    lossy.put("last", lastPoint);
                }

                java.util.List<Map<String, Object>> points = new java.util.ArrayList<>();
                for (ca.team4308.absolutelib.math.trajectories.impl.Pose3d pose : lossyPacket.flightPath) {
                    Map<String, Object> point = new HashMap<>();
                    point.put("x", pose.getTranslation().x);
                    point.put("y", pose.getTranslation().y);
                    point.put("z", pose.getTranslation().z);
                    points.add(point);
                }
                lossy.put("points", points);
            } else {
                lossy.put("count", 0);
                lossy.put("points", java.util.List.of());
            }
            status.put("lossy", lossy);
            ctx.json(status);
        });

        System.out.println("Web Server running on port " + Config.HTTP_PORT);
    }
}
