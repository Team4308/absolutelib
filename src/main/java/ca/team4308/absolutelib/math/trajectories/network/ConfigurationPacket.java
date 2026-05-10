package ca.team4308.absolutelib.math.trajectories.network;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.nio.ByteBuffer;

public class ConfigurationPacket {

    @JsonProperty("config_version_id")
    public int configVersionId;

    @JsonProperty("flywheel_wheel_diameter_inches")
    public double flywheelWheelDiameterInches;

    @JsonProperty("flywheel_compression_ratio")
    public double flywheelCompressionRatio;

    @JsonProperty("flywheel_gear_ratio")
    public double flywheelGearRatio;

    @JsonProperty("flywheel_moment_of_inertia")
    public double flywheelMomentOfInertia;

    @JsonProperty("shooter_pitch_min_degrees")
    public double shooterPitchMinDegrees;

    @JsonProperty("shooter_pitch_max_degrees")
    public double shooterPitchMaxDegrees;

    @JsonProperty("shooter_rpm_min")
    public double shooterRpmMin;

    @JsonProperty("shooter_rpm_max")
    public double shooterRpmMax;

    @JsonProperty("shooter_rpm_to_velocity_factor")
    public double shooterRpmToVelocityFactor;

    @JsonProperty("shooter_distance_min_meters")
    public double shooterDistanceMinMeters;

    @JsonProperty("shooter_distance_max_meters")
    public double shooterDistanceMaxMeters;

    @JsonProperty("shooter_rpm_feedback_threshold")
    public double shooterRpmFeedbackThreshold;

    @JsonProperty("shooter_safety_max_exit_velocity")
    public double shooterSafetyMaxExitVelocity;

    @JsonProperty("solver_pitch_min_degrees")
    public double solverPitchMinDegrees;

    @JsonProperty("solver_pitch_max_degrees")
    public double solverPitchMaxDegrees;

    @JsonProperty("solver_air_resistance_enabled")
    public boolean solverAirResistanceEnabled;

    @JsonProperty("shooter_height_meters")
    public double shooterHeightMeters;

    @JsonProperty("config_timestamp")
    public double configTimestamp;

    public ConfigurationPacket() {
    }


    public static ConfigurationPacket getDefault() {
        ConfigurationPacket config = new ConfigurationPacket();
        config.configVersionId = 1;
        config.flywheelWheelDiameterInches = 4.0;
        config.flywheelCompressionRatio = 0.10;
        config.flywheelGearRatio = 1.0;
        config.flywheelMomentOfInertia = 0.05;
        config.shooterPitchMinDegrees = 47.5;
        config.shooterPitchMaxDegrees = 82.5;
        config.shooterRpmMin = 0;
        config.shooterRpmMax = 6000;
        config.shooterRpmToVelocityFactor = 0.01532;
        config.shooterDistanceMinMeters = 0.5;
        config.shooterDistanceMaxMeters = 12.0;
        config.shooterRpmFeedbackThreshold = 25.0;
        config.shooterSafetyMaxExitVelocity = 99;
        config.solverPitchMinDegrees = 47.5;
        config.solverPitchMaxDegrees = 82.5;
        config.solverAirResistanceEnabled = true;
        config.shooterHeightMeters = 0.5;
        config.configTimestamp = System.currentTimeMillis() / 1000.0;
        return config;
    }

    // Binary Protocol Support (200 bytes)
    public static final int BINARY_SIZE = 200;
    public static final byte MAGIC_BYTE = 0x43; // 'C' for Configuration

    public void toBuffer(ByteBuffer buffer) {
        buffer.putInt(configVersionId);
        buffer.putDouble(flywheelWheelDiameterInches);
        buffer.putDouble(flywheelCompressionRatio);
        buffer.putDouble(flywheelGearRatio);
        buffer.putDouble(flywheelMomentOfInertia);
        buffer.putDouble(shooterPitchMinDegrees);
        buffer.putDouble(shooterPitchMaxDegrees);
        buffer.putDouble(shooterRpmMin);
        buffer.putDouble(shooterRpmMax);
        buffer.putDouble(shooterRpmToVelocityFactor);
        buffer.putDouble(shooterDistanceMinMeters);
        buffer.putDouble(shooterDistanceMaxMeters);
        buffer.putDouble(shooterRpmFeedbackThreshold);
        buffer.putDouble(shooterSafetyMaxExitVelocity);
        buffer.putDouble(solverPitchMinDegrees);
        buffer.putDouble(solverPitchMaxDegrees);
        buffer.put(solverAirResistanceEnabled ? (byte) 1 : (byte) 0);
        buffer.put((byte) 0); // Padding
        buffer.put((byte) 0); // Padding
        buffer.put((byte) 0); // Padding
        buffer.putDouble(shooterHeightMeters);
        buffer.putDouble(configTimestamp);
    }

    public static ConfigurationPacket fromBuffer(ByteBuffer buffer) {
        ConfigurationPacket config = new ConfigurationPacket();
        config.configVersionId = buffer.getInt();
        config.flywheelWheelDiameterInches = buffer.getDouble();
        config.flywheelCompressionRatio = buffer.getDouble();
        config.flywheelGearRatio = buffer.getDouble();
        config.flywheelMomentOfInertia = buffer.getDouble();
        config.shooterPitchMinDegrees = buffer.getDouble();
        config.shooterPitchMaxDegrees = buffer.getDouble();
        config.shooterRpmMin = buffer.getDouble();
        config.shooterRpmMax = buffer.getDouble();
        config.shooterRpmToVelocityFactor = buffer.getDouble();
        config.shooterDistanceMinMeters = buffer.getDouble();
        config.shooterDistanceMaxMeters = buffer.getDouble();
        config.shooterRpmFeedbackThreshold = buffer.getDouble();
        config.shooterSafetyMaxExitVelocity = buffer.getDouble();
        config.solverPitchMinDegrees = buffer.getDouble();
        config.solverPitchMaxDegrees = buffer.getDouble();
        config.solverAirResistanceEnabled = buffer.get() != 0;
        buffer.get(); // Skip padding
        buffer.get(); // Skip padding
        buffer.get(); // Skip padding
        config.shooterHeightMeters = buffer.getDouble();
        config.configTimestamp = buffer.getDouble();
        return config;
    }
}
