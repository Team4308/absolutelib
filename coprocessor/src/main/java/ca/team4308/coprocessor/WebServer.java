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
                                    }
                                    setInterval(fetchStatus, 100);
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
            ctx.json(status);
        });

        System.out.println("Web Server running on port " + Config.HTTP_PORT);
    }
}
