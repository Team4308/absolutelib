package ca.team4308.coprocessor;

import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

import java.util.HashMap;
import java.util.Map;

public class WebServer {

    private final Javalin app;

    public WebServer(TCPServer tcpServer) {
        app = Javalin.create(config -> {
            // No static files needed, just raw handlers
        }).start(Config.HTTP_PORT);

        app.get("/", ctx -> {
            ctx.html("""
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
                        </style>
                        <script>
                            async function fetchStatus() {
                                const res = await fetch('/api/status');
                                const data = await res.json();
                                document.getElementById('connected').innerText = data.isConnected ? "CONNECTED" : "DISCONNECTED";
                                document.getElementById('latency').innerText = data.latencyMs + " ms";
                                
                                if (data.request) {
                                    document.getElementById('robotX').innerText = data.request.robot_x.toFixed(2);
                                    document.getElementById('robotY').innerText = data.request.robot_y.toFixed(2);
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
                    </body>
                    </html>
                    """);
        });

        app.get("/api/status", ctx -> {
            Map<String, Object> status = new HashMap<>();
            status.put("isConnected", tcpServer.isConnected.get());
            status.put("solverTimeMs", tcpServer.lastSolverTimeMs.get());
            status.put("totalRequests", tcpServer.totalRequests.get());
            status.put("droppedPackets", tcpServer.droppedPackets.get());
            status.put("request", tcpServer.latestRequest.get());
            status.put("response", tcpServer.latestResponse.get());
            ctx.json(status);
        });

        System.out.println("Web Server running on port " + Config.HTTP_PORT);
    }
}
