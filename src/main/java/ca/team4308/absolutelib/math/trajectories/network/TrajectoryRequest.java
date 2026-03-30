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

    public TrajectoryRequest() {
    }

    // Binary Protocol Support (88 bytes)
    public static final int BINARY_SIZE = 88;
    public static final byte MAGIC_BYTE = 0x42;

    public void toBuffer(ByteBuffer buffer) {
        buffer.putDouble(timestamp);
        buffer.putDouble(robotX);
        buffer.putDouble(robotY);
        buffer.putDouble(robotHeadingRad);
        buffer.putDouble(vxMps);
        buffer.putDouble(vyMps);
        buffer.putDouble(omegaRadPerSecond);
        buffer.putDouble(targetX);
        buffer.putDouble(targetY);
        buffer.putDouble(targetZ);
        buffer.putDouble(currentRpm);
    }

    public static TrajectoryRequest fromBuffer(ByteBuffer buffer) {
        TrajectoryRequest req = new TrajectoryRequest();
        req.timestamp = buffer.getDouble();
        req.robotX = buffer.getDouble();
        req.robotY = buffer.getDouble();
        req.robotHeadingRad = buffer.getDouble();
        req.vxMps = buffer.getDouble();
        req.vyMps = buffer.getDouble();
        req.omegaRadPerSecond = buffer.getDouble();
        req.targetX = buffer.getDouble();
        req.targetY = buffer.getDouble();
        req.targetZ = buffer.getDouble();
        req.currentRpm = buffer.getDouble();
        return req;
    }
}
