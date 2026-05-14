package ca.team4308.absolutelib.math.trajectories.network;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonProperty;

import ca.team4308.absolutelib.math.trajectories.impl.Pose3d;
import ca.team4308.absolutelib.math.trajectories.impl.Rotation3d;

public class LossyDataPacket {
    @JsonProperty("flight_path")
    public List<Pose3d> flightPath = new ArrayList<>();


    public LossyDataPacket() {
        
    }

    // We don't really care if this packet is lost as it transfers non essential data.
    public static final int MAX_POINTS = 25;
    public static final int BYTES_PER_POINT = Double.BYTES * 3; // x, y, z
    public static final int BINARY_SIZE = 1 + 1 + (MAX_POINTS * BYTES_PER_POINT); // magic + count + payload
    public static final byte MAGIC_BYTE = 0x4c; // L for Lossy

    public void toBuffer(ByteBuffer buffer) {
        buffer.put(MAGIC_BYTE);
        int count = Math.min(flightPath.size(), MAX_POINTS);
        buffer.put((byte) count);

        // We will only send the position (x, y, z) of each Pose3d to save space, ignoring rotation.
        for (int i = 0; i < MAX_POINTS; i++) {
            if (i < count) {
                Pose3d pose = flightPath.get(i);
                buffer.putDouble(pose.getTranslation().x);
                buffer.putDouble(pose.getTranslation().y);
                buffer.putDouble(pose.getTranslation().z);
            } else {
                buffer.putDouble(0.0);
                buffer.putDouble(0.0);
                buffer.putDouble(0.0);
            }
        }
    }

    public static LossyDataPacket fromBuffer(ByteBuffer buffer) {
        // Assume magic byte has been read
        LossyDataPacket packet = new LossyDataPacket();
        int count = Byte.toUnsignedInt(buffer.get());
        for (int i = 0; i < MAX_POINTS; i++) {
            double x = buffer.getDouble();
            double y = buffer.getDouble();
            double z = buffer.getDouble();
            if (i < count) {
                packet.flightPath.add(new Pose3d(x, y, z, new Rotation3d()));
            }
        }
        return packet;
    }

}
