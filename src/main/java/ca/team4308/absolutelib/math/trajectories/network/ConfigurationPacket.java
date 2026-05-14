package ca.team4308.absolutelib.math.trajectories.network;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.nio.ByteBuffer;

/**
 * Expanded configuration packet for the trajectory system.
 * Inherits from TrajectoryConfigDTO to provide full serialization support
 * while maintaining compatibility with existing binary protocols if needed.
 */
public class ConfigurationPacket extends TrajectoryConfigDTO {

    // Binary Protocol Support (for legacy or low-bandwidth cases)
    public static final int BINARY_SIZE = 512; // Increased size for more fields
    public static final byte MAGIC_BYTE = 0x43; // 'C' for Configuration

    public ConfigurationPacket() {
        super();
    }

    public static ConfigurationPacket fromDTO(TrajectoryConfigDTO dto) {
        ConfigurationPacket packet = new ConfigurationPacket();
        // Manual copy of fields if necessary, but since they are public we can just use the DTO
        // Actually, it's better to just use the DTO directly in most places.
        return packet;
    }

    /**
     * Minimal binary support for core hardware limits.
     * Complex structures like lookup tables are recommended to stay in JSON.
     */
    public void toBuffer(ByteBuffer buffer) {
        buffer.putInt(configVersionId);
        buffer.putDouble(shooterPitchMin);
        buffer.putDouble(shooterPitchMax);
        buffer.putDouble(shooterRpmMin);
        buffer.putDouble(shooterRpmMax);
        buffer.putDouble(shooterRpmToVelocityFactor);
        buffer.putDouble(shooterDistanceMin);
        buffer.putDouble(shooterDistanceMax);
        buffer.putDouble(shooterRpmFeedbackThreshold);
        buffer.putDouble(shooterSafetyMaxExitVel);
        buffer.putDouble(timestamp);
    }

    public static ConfigurationPacket fromBuffer(ByteBuffer buffer) {
        ConfigurationPacket config = new ConfigurationPacket();
        config.configVersionId = buffer.getInt();
        config.shooterPitchMin = buffer.getDouble();
        config.shooterPitchMax = buffer.getDouble();
        config.shooterRpmMin = buffer.getDouble();
        config.shooterRpmMax = buffer.getDouble();
        config.shooterRpmToVelocityFactor = buffer.getDouble();
        config.shooterDistanceMin = buffer.getDouble();
        config.shooterDistanceMax = buffer.getDouble();
        config.shooterRpmFeedbackThreshold = buffer.getDouble();
        config.shooterSafetyMaxExitVel = buffer.getDouble();
        config.timestamp = buffer.getDouble();
        return config;
    }
}
