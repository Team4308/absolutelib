package ca.team4308.absolutelib.math.trajectories.network;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class TrajectoryRequest {

    // Unique timestamp or ID for the request to match with responses
    @JsonProperty("timestamp")
    public double timestamp;

    // Robot pose
    @JsonProperty("robot_x")
    public double robotX;

    @JsonProperty("robot_y")
    public double robotY;

    @JsonProperty("robot_z")
    public double robotZ;

    @JsonProperty("robot_heading_rad")
    public double robotHeadingRad;

    // Chassis velocity
    @JsonProperty("vx_mps")
    public double vxMps;

    @JsonProperty("vy_mps")
    public double vyMps;

    @JsonProperty("omega_rad_per_second")
    public double omegaRadPerSecond;

    // Target info
    @JsonProperty("target_x")
    public double targetX;

    @JsonProperty("target_y")
    public double targetY;
    
    @JsonProperty("target_z")
    public double targetZ;

    // Current RPM (for readiness check / friction models)
    @JsonProperty("current_rpm")
    public double currentRpm;

    @JsonProperty("active_rpm")
    public double activeRpm;

    @JsonProperty("active_pitch_deg")
    public double activePitchDegrees;

    // Optional battery voltage / percentage from robot
    @JsonProperty("battery")
    public double battery = 100.0;

    public TrajectoryRequest() {
    }

    // Binary Protocol Support (112 bytes)
    public static final int BINARY_SIZE = 112;
    public static final byte MAGIC_BYTE = 0x42;

    public void toBuffer(ByteBuffer buffer) {
        buffer.putDouble(timestamp);
        buffer.putDouble(robotX);
        buffer.putDouble(robotY);
    buffer.putDouble(robotZ);
        buffer.putDouble(robotHeadingRad);
        buffer.putDouble(vxMps);
        buffer.putDouble(vyMps);
        buffer.putDouble(omegaRadPerSecond);
        buffer.putDouble(targetX);
        buffer.putDouble(targetY);
        buffer.putDouble(targetZ);
        buffer.putDouble(currentRpm);
        buffer.putDouble(activeRpm);
        buffer.putDouble(activePitchDegrees);
    }

    public static TrajectoryRequest fromBuffer(ByteBuffer buffer) {
        TrajectoryRequest req = new TrajectoryRequest();
        req.timestamp = buffer.getDouble();
        req.robotX = buffer.getDouble();
        req.robotY = buffer.getDouble();
    req.robotZ = buffer.getDouble();
        req.robotHeadingRad = buffer.getDouble();
        req.vxMps = buffer.getDouble();
        req.vyMps = buffer.getDouble();
        req.omegaRadPerSecond = buffer.getDouble();
        req.targetX = buffer.getDouble();
        req.targetY = buffer.getDouble();
        req.targetZ = buffer.getDouble();
        req.currentRpm = buffer.getDouble();
        req.activeRpm = buffer.getDouble();
        req.activePitchDegrees = buffer.getDouble();
        // Buffer format; keep safe default if no extra data.
        if (buffer.remaining() >= Double.BYTES) {
            req.battery = buffer.getDouble();
        }
        return req;
    }
}
