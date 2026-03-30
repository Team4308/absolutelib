package ca.team4308.absolutelib.math.trajectories.network;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class TrajectoryResponse {

    @JsonProperty("timestamp")
    public double timestamp;

    @JsonProperty("valid")
    public boolean valid;

    @JsonProperty("pitch_deg")
    public double pitchDegrees;

    @JsonProperty("yaw_deg")
    public double yawDegrees;

    @JsonProperty("rpm")
    public double rpm;

    @JsonProperty("time_of_flight_sec")
    public double timeOfFlightSec;

    @JsonProperty("confidence")
    public double confidence;

    @JsonProperty("status")
    public String status;

    @JsonProperty("ready_to_fire")
    public boolean readyToFire;

    public TrajectoryResponse() {
    }

    // Binary Protocol Support (51 bytes) 1 + 8 + 1 + 8 + 8 + 8 + 8 + 8 + 1
    public static final int BINARY_SIZE = 51;
    public static final byte MAGIC_BYTE = 0x42;

    public void toBuffer(ByteBuffer buffer) {
        buffer.put(MAGIC_BYTE);
        buffer.putDouble(timestamp);
        buffer.put((byte)(valid ? 1 : 0));
        buffer.putDouble(pitchDegrees);
        buffer.putDouble(yawDegrees);
        buffer.putDouble(rpm);
        buffer.putDouble(timeOfFlightSec);
        buffer.putDouble(confidence);
        buffer.put((byte)(readyToFire ? 1 : 0));
    }

    public static TrajectoryResponse fromBuffer(ByteBuffer buffer) {
        // Assume magic byte has been read
        TrajectoryResponse res = new TrajectoryResponse();
        res.timestamp = buffer.getDouble();
        res.valid = buffer.get() == 1;
        res.pitchDegrees = buffer.getDouble();
        res.yawDegrees = buffer.getDouble();
        res.rpm = buffer.getDouble();
        res.timeOfFlightSec = buffer.getDouble();
        res.confidence = buffer.getDouble();
        res.readyToFire = buffer.get() == 1;
        // Status string is omitted in binary proto but can be interpolated
        res.status = res.valid ? "OK" : "INVALID"; 
        return res;
    }
}
