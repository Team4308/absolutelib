package ca.team4308.absolutelib.math.trajectories.flywheel;

import ca.team4308.absolutelib.math.trajectories.motor.MotorSpec;
import ca.team4308.absolutelib.math.trajectories.motor.FRCMotors;
import ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants;
import edu.wpi.first.math.util.Units;

/**
 * Configuration for a flywheel shooter mechanism.
 * Defines wheel geometry, material, compression, and motor setup.
 */
public class FlywheelConfig {
    
    /**
     * Flywheel arrangement type.
     */
    public enum WheelArrangement {
        SINGLE,          // Single flywheel, ball rolls on surface
        DUAL_PARALLEL,   // Two flywheels side by side
        DUAL_OVER_UNDER, // Top and bottom flywheels (most common)
        STACKED,         // Multiple wheels on same shaft
        DIFFERENTIAL     // Flywheels at different speeds for spin
    }
    
    private final String name;
    private final WheelArrangement arrangement;
    private final double wheelDiameterMeters;
    private final double wheelWidthMeters;
    private final WheelMaterial material;
    private final double compressionRatio;     // How much ball is compressed (0-0.3 typical)
    private final int wheelCount;
    private final MotorSpec motor;
    private final int motorsPerWheel;
    private final double gearRatio;            // Motor:Wheel ratio (>1 = geared down)
    
    /**
     * Creates a flywheel configuration.
     */
    public FlywheelConfig(String name, WheelArrangement arrangement,
                          double wheelDiameterMeters, double wheelWidthMeters,
                          WheelMaterial material, double compressionRatio,
                          int wheelCount, MotorSpec motor, int motorsPerWheel,
                          double gearRatio) {
        this.name = name;
        this.arrangement = arrangement;
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.wheelWidthMeters = wheelWidthMeters;
        this.material = material;
        this.compressionRatio = compressionRatio;
        this.wheelCount = wheelCount;
        this.motor = motor;
        this.motorsPerWheel = motorsPerWheel;
        this.gearRatio = gearRatio;
    }
    
    /**
     * Builder for fluent configuration.
     */
    public static class Builder {
        private String name = "Custom Flywheel";
        private WheelArrangement arrangement = WheelArrangement.DUAL_OVER_UNDER;
        private double wheelDiameterInches = 4.0;
        private double wheelWidthInches = 2.0;
        private WheelMaterial material = WheelMaterial.GREEN_COMPLIANT;
        private double compressionRatio = 0.10;
        private int wheelCount = 2;
        private MotorSpec motor = FRCMotors.KRAKEN_X60;
        private int motorsPerWheel = 1;
        private double gearRatio = 1.0;
        
        public Builder name(String name) {
            this.name = name;
            return this;
        }
        
        public Builder arrangement(WheelArrangement arrangement) {
            this.arrangement = arrangement;
            return this;
        }
        
        public Builder wheelDiameterInches(double inches) {
            this.wheelDiameterInches = inches;
            return this;
        }
        
        public Builder wheelWidthInches(double inches) {
            this.wheelWidthInches = inches;
            return this;
        }
        
        public Builder material(WheelMaterial material) {
            this.material = material;
            return this;
        }
        
        public Builder durometer(int durometer) {
            this.material = WheelMaterial.custom("Custom " + durometer + "A", durometer);
            return this;
        }
        
        public Builder compressionRatio(double ratio) {
            this.compressionRatio = ratio;
            return this;
        }
        
        public Builder compressionInches(double inches, double ballDiameterInches) {
            this.compressionRatio = inches / ballDiameterInches;
            return this;
        }
        
        public Builder wheelCount(int count) {
            this.wheelCount = count;
            return this;
        }
        
        public Builder motor(MotorSpec motor) {
            this.motor = motor;
            return this;
        }
        
        public Builder motorsPerWheel(int count) {
            this.motorsPerWheel = count;
            return this;
        }
        
        public Builder gearRatio(double ratio) {
            this.gearRatio = ratio;
            return this;
        }
        
        public FlywheelConfig build() {
            return new FlywheelConfig(
                name, arrangement,
                Units.inchesToMeters(wheelDiameterInches),
                Units.inchesToMeters(wheelWidthInches),
                material, compressionRatio, wheelCount,
                motor, motorsPerWheel, gearRatio
            );
        }
    }
    
    public static Builder builder() {
        return new Builder();
    }
    
    // Derived properties
    
    /**
     * Gets wheel radius in meters.
     */
    public double getWheelRadiusMeters() {
        return wheelDiameterMeters / 2.0;
    }
    
    /**
     * Gets wheel diameter in inches.
     */
    public double getWheelDiameterInches() {
        return Units.metersToInches(wheelDiameterMeters);
    }
    
    /**
     * Calculates moment of inertia for a single wheel (solid cylinder approximation).
     */
    public double getSingleWheelInertia() {
        double radius = getWheelRadiusMeters();
        double volume = Math.PI * radius * radius * wheelWidthMeters;
        double mass = volume * material.getDensity();
        
        // Moment of inertia for solid cylinder: I = 0.5 * m * r^2
        return 0.5 * mass * radius * radius;
    }
    
    /**
     * Calculates total flywheel inertia including all wheels and motor rotors.
     */
    public double getTotalInertia() {
        double wheelInertia = getSingleWheelInertia() * wheelCount;
        double motorInertia = motor.getRotorInertiaKgM2() * wheelCount * motorsPerWheel;
        
        // Motor inertia is reflected through gear ratio^2
        double reflectedMotorInertia = motorInertia * gearRatio * gearRatio;
        
        return wheelInertia + reflectedMotorInertia;
    }
    
    /**
     * Calculates maximum achievable wheel RPM based on motor and gearing.
     */
    public double getMaxWheelRpm() {
        return motor.getFreeSpeedRpm() / gearRatio;
    }
    
    /**
     * Calculates surface velocity at a given wheel RPM.
     */
    public double getSurfaceVelocity(double wheelRpm) {
        double angularVelocity = PhysicsConstants.rpmToRadPerSec(wheelRpm);
        return angularVelocity * getWheelRadiusMeters();
    }
    
    /**
     * Calculates the RPM needed for a target surface velocity.
     */
    public double getRpmForVelocity(double velocityMps) {
        double angularVelocity = velocityMps / getWheelRadiusMeters();
        return PhysicsConstants.radPerSecToRpm(angularVelocity);
    }
    
    /**
     * Calculates motor RPM for a given wheel RPM.
     */
    public double getMotorRpmForWheelRpm(double wheelRpm) {
        return wheelRpm * gearRatio;
    }
    
    /**
     * Estimates available torque at a given wheel RPM.
     */
    public double getTorqueAtWheelRpm(double wheelRpm) {
        double motorRpm = getMotorRpmForWheelRpm(wheelRpm);
        double motorTorque = motor.getTorqueAtRpm(motorRpm);
        
        // Torque multiplied by gear ratio and number of motors
        return motorTorque * gearRatio * motorsPerWheel * wheelCount;
    }
    
    /**
     * Estimates spin-up time to reach target RPM.
     */
    public double estimateSpinUpTime(double targetWheelRpm) {
        double totalInertia = getTotalInertia();
        double motorRpm = getMotorRpmForWheelRpm(targetWheelRpm);
        
        return motor.estimateSpinUpTime(motorRpm, totalInertia / (gearRatio * gearRatio));
    }
    
    /**
     * Calculates stored kinetic energy at a given RPM.
     */
    public double getStoredEnergy(double wheelRpm) {
        double omega = PhysicsConstants.rpmToRadPerSec(wheelRpm);
        double inertia = getTotalInertia();
        
        // KE = 0.5 * I * omega^2
        return 0.5 * inertia * omega * omega;
    }
    
    /**
     * Estimates current draw at a given wheel RPM.
     */
    public double estimateCurrentDraw(double wheelRpm) {
        double motorRpm = getMotorRpmForWheelRpm(wheelRpm);
        return motor.getCurrentAtRpm(motorRpm) * motorsPerWheel * wheelCount;
    }
    
    // Getters
    public String getName() { return name; }
    public WheelArrangement getArrangement() { return arrangement; }
    public double getWheelDiameterMeters() { return wheelDiameterMeters; }
    public double getWheelWidthMeters() { return wheelWidthMeters; }
    public WheelMaterial getMaterial() { return material; }
    public double getCompressionRatio() { return compressionRatio; }
    public int getWheelCount() { return wheelCount; }
    public MotorSpec getMotor() { return motor; }
    public int getMotorsPerWheel() { return motorsPerWheel; }
    public double getGearRatio() { return gearRatio; }
    
    @Override
    public String toString() {
        return String.format("%s: %.1f\" %s wheels, %dA durometer, %.1f:1 ratio, %s motors",
            name, getWheelDiameterInches(), arrangement, material.getDurometer(),
            gearRatio, motor.getName());
    }
}
