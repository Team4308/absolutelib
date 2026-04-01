# Trajectory Coprocessor

This is a standalone runtime for the `TrajectorySolver`. It runs on a Raspberry Pi, Orange Pi, or any Linux machine, and communicates with the roboRIO over TCP and NetworkTables 4.

> **Note on Solver Logic:** The existing trajectory solver logic remains entirely unchanged. The coprocessor only wraps and invokes it through a network-facing interface to offload the compute cost from the roboRIO.

## Coordinate System & Units
For accurate physics and integration, the coprocessor strictly adheres to standard WPILib SI units and field-relative conventions:
- **Distance/Position:** Meters (m)
- **Velocity:** Meters per second (m/s)
- **Angles/Rotation:** Radians (rad) internally and in JSON requests (yaw/heading), but responses output Degrees for pitch/yaw for easier physical shooter use.
- **Spin/Flywheel Speed:** Revolutions per minute (RPM).
- **Time:** Seconds (s).
- **Reference Frame:** **Field-relative** for both target and robot pose. Calculations assume standard WPILib blue-alliance origin (X+ points toward red alliance, Y+ points left).

## Message Framing & Protocol
Communication occurs over a raw TCP socket (default port 5801). The protocol uses **newline-delimited JSON**.
Every message sent by the robot must be a single JSON object followed immediately by a newline (`\n`). The coprocessor responds in kind.

### Example Request (`TrajectoryRequest`)
*The robot must send field-relative pose, velocity, and targets.*
```json
{
  "timestamp": 1710000000.000,
  "robot_x": 2.1,
  "robot_y": 4.3,
  "robot_heading_rad": 1.570796,
  "vx_mps": 1.2,
  "vy_mps": -0.3,
  "omega_rad_per_second": 0.5,
  "target_x": 8.0,
  "target_y": 4.0,
  "target_z": 2.5,
  "current_rpm": 2000.0
}
```

### Example Response (`TrajectoryResponse`)
```json
{
  "timestamp": 1710000000.000,
  "valid": true,
  "pitch_deg": 17.4,
  "yaw_deg": 0.0,
  "rpm": 2100.0,
  "time_of_flight_sec": 0.42,
  "confidence": 0.97,
  "status": "OK",
  "ready_to_fire": true
}
```

## Freshness, Stale Data & Failure Behavior
Network communication involves inherent risks (latency spikes, lost packets, disconnected cables). 

**Freshness Policy:**
- In `ExampleShooter.java`, data is considered **stale** if `(Current_FPGATimestamp - response.timestamp) > 0.5` seconds.
- *Note:* `0.5s` is a conservative default. For high-speed shooting in competition, you should tune this down to `0.1s` or lower.
- The `CoprocessorClient` will attempt to automatically reconnect if the socket is closed or the read buffer returns `null`, waiting `1s` between reconnection attempts.

**Fallback Behavior:**
- **When Fallback Starts:** Immediately when the connection drops, or a packet hasn't been received in time (stale), or the response is explicitly invalid.
- **When Fallback Stops:** As soon as the connection is re-established and a fresh, valid response (`timestamp` within threshold) is received, the RIO synchronizes its manual override and uses the TCP data.
- **Alerts:** The RIO logs `/Shooter/FallbackActive` to NT4/AdvantageKit. When fallback triggers, it logs `/Shooter/FallbackReason_StaleTime` to help debug if it dropped due to latency or a hard disconnect. There is currently no delay hysteresis; recovery happens immediately on the next valid packet.

## Web Dashboard & API Endpoints
The coprocessor hosts a Javalin web server on port `5800`.
- **`GET /`**: Serves a user-friendly HTML dashboard showing realtime connection status, latency, and solver states.
- **`GET /api/status`**: Returns a JSON object containing `isConnected`, `latencyMs`, `request`, and `response`.

## NT4 Tables & Keys
Publishing is strictly for Shuffleboard / AdvantageScope debugging. The NT4 instance publishes to table `/TrajectoryCoprocessor/`:
- `/TrajectoryCoprocessor/Status/TCPLatencyMs`
- `/TrajectoryCoprocessor/Input/RobotX`
- `/TrajectoryCoprocessor/Input/RobotY`
- `/TrajectoryCoprocessor/Output/PitchDeg`
- `/TrajectoryCoprocessor/Output/YawDeg`
- `/TrajectoryCoprocessor/Output/RPM`
- `/TrajectoryCoprocessor/Output/Valid`

## Threading Model
- **TCP Server Thread**: Dedicated thread to handle the single RIO client connection. Uses blocking I/O wrapped in `BufferedReader.readLine()` to prevent high CPU spin.
- **Web Server Thread**: Managed by Javalin's internal Jetty thread pool.
- **Main Loop (NT4 Publish)**: The main Java thread loops at 50Hz to copy atomic references from the TCP thread into NetworkTables.

## Performance Measurement
The latency value reported on the dashboard is measured server-side using `System.currentTimeMillis()` diff before and after the `solver.solve()` call. This only represents calculation time.
True end-to-end latency (RTT) is measured by the RoboRIO in `ExampleShooter.java` using `(Timer.getFPGATimestamp() - res.timestamp)` and is broadcast to `/Shooter/CoprocessorRTT_ms`. It typically sits well below 100ms depending on radio/switch load.

## Localhost vs Hardware Mode Switching
The coprocessor's host IP can be changed directly inside the robot code.
By default in `ExampleShooter.java` in the 2026 example SRC:
- If `RobotBase.isSimulation()` -> Connects to `127.0.0.1:5801`.
- If `RobotBase.isReal()` -> Connects to `10.43.8.77:5801`.
The Coprocessor's own logic uses the `IS_SIMULATION` env variable to deduce the NT4 server location (`10.43.8.2` or `127.0.0.1`).

## Versioning & Build Info
- The build produces a JAR at: `coprocessor/build/libs/coprocessor.jar`
- Currently, the version hash is statically reported as `v1.0.0` in the web dashboard. For full CI/CD, the `.github/workflows` script or gradle config should inject the git commit hash.

## SystemD Service Setup
To deploy on the Pi, deploy the JAR and run it as an auto-restarting service.

Create `/etc/systemd/system/trajectory-coprocessor.service`:
```ini
[Unit]
Description=Trajectory Coprocessor
After=network.target

[Service]
Type=simple
User=frc
WorkingDirectory=/home/frc/trajectory-coprocessor
ExecStart=/usr/bin/java -jar /home/frc/trajectory-coprocessor/coprocessor.jar
Environment="TEAM_NUMBER=4308"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```
Start and monitor logs using `sudo journalctl -u trajectory-coprocessor -f`. 

## Logging
- All standard coprocessor events (`Client connected`, `Error processing request`, etc.) are written directly to stdout `System.out`. 
- When run via SystemD, these logs are automatically captured by the Linux `journalctl` daemon.
- The Robot logs active RTT and Fallback reasons strongly via AdvantageKit logger (`Logger.recordOutput`).

## Known Limitations
- JSON serialization is used over a binary protocol. Binary (like raw bytes or protobuf) could be implemented for marginally faster network times.
- Localhost testing does not accurately reflect FRC radio competition-latency spikes.
- Fallback threshold may still need tuning. The current protocol is v1 and is subject to change.
