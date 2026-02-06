package ca.team4308.absolutelib.math.trajectories;

import edu.wpi.first.math.util.Units;

/**
 * Input parameters for a trajectory shot calculation.
 * Immutable data class representing the shot setup.
 */
public class ShotInput {
    
    /**
     * Shot selection preference when multiple candidates are available.
     */
    public enum ShotPreference {
        /** Automatically select best overall candidate */
        AUTO,
        /** Prefer fastest time of flight (low arc) */
        FASTEST,
        /** Prefer highest clearance (high arc) */
        HIGH_CLEARANCE,
        /** Prefer most stable/consistent shot */
        MOST_STABLE,
        /** Prefer minimum velocity (easiest on mechanism) */
        MIN_VELOCITY,
        /** Prefer maximum accuracy (closest to target center) */
        MOST_ACCURATE,
        /** Legacy: prefer high arc trajectories */
        @Deprecated
        PREFER_HIGH_ARC,
        /** Legacy: prefer low arc trajectories */
        @Deprecated
        PREFER_LOW_ARC
    }
    
    // Shooter position (meters)
    private final double shooterX;
    private final double shooterY;
    private final double shooterZ;
    
    // Shooter orientation (radians)
    private final double shooterYaw;
    
    // Target position (meters)
    private final double targetX;
    private final double targetY;
    private final double targetZ;
    
    // Target acceptance radius (meters)
    private final double targetRadius;
    
    // Robot velocity (meters/sec)
    private final double robotVx;
    private final double robotVy;
    
    // Shot preferences
    private final boolean preferHighArc;  // Legacy support
    private final boolean includeAirResistance;
    
    // New preferences
    private final ShotPreference shotPreference;
    private final int maxCandidates;
    private final double minPitchDegrees;
    private final double maxPitchDegrees;
    private final double minVelocityMps;
    private final double maxVelocityMps;
    private final double angleStepDegrees;
    private final double minArcHeightMeters; // Minimum apex height above target
    
    /**
     * Creates a shot input with all parameters (legacy constructor).
     * @deprecated Use the Builder instead for full control over all parameters.
     */
    @Deprecated
    public ShotInput(double shooterX, double shooterY, double shooterZ,
                     double shooterYaw,
                     double targetX, double targetY, double targetZ,
                     double targetRadius,
                     double robotVx, double robotVy,
                     boolean preferHighArc, boolean includeAirResistance) {
        this(shooterX, shooterY, shooterZ, shooterYaw,
             targetX, targetY, targetZ, targetRadius,
             robotVx, robotVy, preferHighArc, includeAirResistance,
             preferHighArc ? ShotPreference.PREFER_HIGH_ARC : ShotPreference.PREFER_LOW_ARC,
             50, 0, 90, 5, 50, 1.0, 0.0);
    }
    
    /**
     * Full constructor with all new parameters.
     */
    private ShotInput(double shooterX, double shooterY, double shooterZ,
                      double shooterYaw,
                      double targetX, double targetY, double targetZ,
                      double targetRadius,
                      double robotVx, double robotVy,
                      boolean preferHighArc, boolean includeAirResistance,
                      ShotPreference shotPreference, int maxCandidates,
                      double minPitchDegrees, double maxPitchDegrees,
                      double minVelocityMps, double maxVelocityMps,
                      double angleStepDegrees, double minArcHeightMeters) {
        this.shooterX = shooterX;
        this.shooterY = shooterY;
        this.shooterZ = shooterZ;
        this.shooterYaw = shooterYaw;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetZ = targetZ;
        this.targetRadius = targetRadius;
        this.robotVx = robotVx;
        this.robotVy = robotVy;
        this.preferHighArc = preferHighArc;
        this.includeAirResistance = includeAirResistance;
        this.shotPreference = shotPreference;
        this.maxCandidates = maxCandidates;
        this.minPitchDegrees = minPitchDegrees;
        this.maxPitchDegrees = maxPitchDegrees;
        this.minVelocityMps = minVelocityMps;
        this.maxVelocityMps = maxVelocityMps;
        this.angleStepDegrees = angleStepDegrees;
        this.minArcHeightMeters = minArcHeightMeters;
    }
    
    /**
     * Builder for fluent construction with unit flexibility.
     */
    public static class Builder {
        private double shooterX = 0;
        private double shooterY = 0;
        private double shooterZ = 0;
        private double shooterYaw = 0;
        private double targetX = 0;
        private double targetY = 0;
        private double targetZ = 0;
        private double targetRadius = 0.15; // 15cm default
        private double robotVx = 0;
        private double robotVy = 0;
        private boolean preferHighArc = true;
        private boolean includeAirResistance = true;
        
        // New parameters
        private ShotPreference shotPreference = ShotPreference.AUTO;
        private int maxCandidates = 50;
        private double minPitchDegrees = 5;
        private double maxPitchDegrees = 85;
        private double minVelocityMps = 5;
        private double maxVelocityMps = 50;
        private double angleStepDegrees = 1.0;  // 1 degree steps = O(80) iterations max
        private double minArcHeightMeters = 0.0; // 0 = no minimum arc height requirement
        
        /**
         * Sets shooter position in meters.
         */
        public Builder shooterPositionMeters(double x, double y, double z) {
            this.shooterX = x;
            this.shooterY = y;
            this.shooterZ = z;
            return this;
        }
        
        /**
         * Sets shooter position in inches.
         */
        public Builder shooterPositionInches(double x, double y, double z) {
            this.shooterX = Units.inchesToMeters(x);
            this.shooterY = Units.inchesToMeters(y);
            this.shooterZ = Units.inchesToMeters(z);
            return this;
        }
        
        /**
         * Sets shooter position in feet.
         */
        public Builder shooterPositionFeet(double x, double y, double z) {
            this.shooterX = Units.feetToMeters(x);
            this.shooterY = Units.feetToMeters(y);
            this.shooterZ = Units.feetToMeters(z);
            return this;
        }
        
        /**
         * Sets shooter yaw in radians.
         */
        public Builder shooterYawRadians(double yaw) {
            this.shooterYaw = yaw;
            return this;
        }
        
        /**
         * Sets shooter yaw in degrees.
         */
        public Builder shooterYawDegrees(double yaw) {
            this.shooterYaw = Math.toRadians(yaw);
            return this;
        }
        
        /**
         * Sets target position in meters.
         */
        public Builder targetPositionMeters(double x, double y, double z) {
            this.targetX = x;
            this.targetY = y;
            this.targetZ = z;
            return this;
        }
        
        /**
         * Sets target position in inches.
         */
        public Builder targetPositionInches(double x, double y, double z) {
            this.targetX = Units.inchesToMeters(x);
            this.targetY = Units.inchesToMeters(y);
            this.targetZ = Units.inchesToMeters(z);
            return this;
        }
        
        /**
         * Sets target position in feet.
         */
        public Builder targetPositionFeet(double x, double y, double z) {
            this.targetX = Units.feetToMeters(x);
            this.targetY = Units.feetToMeters(y);
            this.targetZ = Units.feetToMeters(z);
            return this;
        }
        
        /**
         * Sets target acceptance radius in meters.
         */
        public Builder targetRadiusMeters(double radius) {
            this.targetRadius = radius;
            return this;
        }
        
        /**
         * Sets target acceptance radius in inches.
         */
        public Builder targetRadiusInches(double radius) {
            this.targetRadius = Units.inchesToMeters(radius);
            return this;
        }
        
        /**
         * Sets robot velocity in meters/sec (field-relative).
         */
        public Builder robotVelocity(double vx, double vy) {
            this.robotVx = vx;
            this.robotVy = vy;
            return this;
        }
        
        /**
         * Sets whether to prefer high-arc trajectories.
         * @deprecated Use {@link #shotPreference(ShotPreference)} instead
         */
        @Deprecated
        public Builder preferHighArc(boolean prefer) {
            this.preferHighArc = prefer;
            this.shotPreference = prefer ? ShotPreference.PREFER_HIGH_ARC : ShotPreference.PREFER_LOW_ARC;
            return this;
        }
        
        /**
         * Sets whether to include air resistance.
         */
        public Builder includeAirResistance(boolean include) {
            this.includeAirResistance = include;
            return this;
        }
        
        /**
         * Sets the shot preference for candidate selection.
         * This replaces the old preferHighArc boolean.
         */
        public Builder shotPreference(ShotPreference preference) {
            this.shotPreference = preference;
            // Update legacy field for backward compatibility
            this.preferHighArc = (preference == ShotPreference.PREFER_HIGH_ARC || 
                                  preference == ShotPreference.HIGH_CLEARANCE);
            return this;
        }
        
        /**
         * Sets the maximum number of candidates to generate.
         * Higher values give more options but take longer.
         * Default: 50
         */
        public Builder maxCandidates(int max) {
            this.maxCandidates = Math.max(1, max);
            return this;
        }
        
        /**
         * Sets the pitch angle range in degrees.
         * Default: 5 to 85 degrees
         */
        public Builder pitchRangeDegrees(double min, double max) {
            this.minPitchDegrees = Math.max(0.1, min);
            this.maxPitchDegrees = Math.min(89.9, max);
            return this;
        }
        
        /**
         * Sets the velocity range in meters per second.
         * Default: 5 to 50 m/s
         */
        public Builder velocityRangeMps(double min, double max) {
            this.minVelocityMps = Math.max(0.1, min);
            this.maxVelocityMps = max;
            return this;
        }
        
        /**
         * Sets the angle step size for candidate generation.
         * Smaller values = more candidates, higher precision.
         * Default: 1.0 degrees (results in ~80 candidates max)
         */
        public Builder angleStepDegrees(double step) {
            this.angleStepDegrees = Math.max(0.1, step);
            return this;
        }
        
        /**
         * Sets the minimum arc height above the target.
         * This ensures the ball reaches a certain apex height before descending.
         * Higher values create more pronounced arcs that drop into the target.
         * Default: 0.0 (no minimum)
         * @param meters Minimum height above target that the trajectory apex must reach
         */
        public Builder minArcHeightMeters(double meters) {
            this.minArcHeightMeters = Math.max(0.0, meters);
            return this;
        }
        
        /**
         * Sets the minimum arc height above the target in feet.
         * @param feet Minimum height above target in feet
         */
        public Builder minArcHeightFeet(double feet) {
            this.minArcHeightMeters = Math.max(0.0, Units.feetToMeters(feet));
            return this;
        }
        
        /**
         * Configures for fast solving with fewer candidates.
         */
        public Builder fastMode() {
            this.angleStepDegrees = 2.0;
            this.maxCandidates = 20;
            return this;
        }
        
        /**
         * Configures for high precision with more candidates.
         */
        public Builder precisionMode() {
            this.angleStepDegrees = 0.5;
            this.maxCandidates = 100;
            return this;
        }
        
        public ShotInput build() {
            return new ShotInput(
                shooterX, shooterY, shooterZ,
                shooterYaw,
                targetX, targetY, targetZ,
                targetRadius,
                robotVx, robotVy,
                preferHighArc, includeAirResistance,
                shotPreference, maxCandidates,
                minPitchDegrees, maxPitchDegrees,
                minVelocityMps, maxVelocityMps,
                angleStepDegrees, minArcHeightMeters
            );
        }
    }
    
    public static Builder builder() {
        return new Builder();
    }
    
    // Derived properties
    
    /**
     * Calculates horizontal distance from shooter to target.
     */
    public double getHorizontalDistanceMeters() {
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
     * Calculates 3D distance from shooter to target.
     */
    public double getTotalDistanceMeters() {
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
        double dz = targetZ - shooterZ;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    /**
     * Calculates height difference (target - shooter).
     */
    public double getHeightDifferenceMeters() {
        return targetZ - shooterZ;
    }
    
    /**
     * Calculates required yaw to face target.
     */
    public double getRequiredYawRadians() {
        return Math.atan2(targetY - shooterY, targetX - shooterX);
    }
    
    /**
     * Calculates yaw adjustment needed from current shooter yaw.
     */
    public double getYawAdjustmentRadians() {
        double required = getRequiredYawRadians();
        double adjustment = required - shooterYaw;
        
        // Normalize to -PI to PI
        while (adjustment > Math.PI) adjustment -= 2 * Math.PI;
        while (adjustment < -Math.PI) adjustment += 2 * Math.PI;
        
        return adjustment;
    }
    
    // Getters
    public double getShooterX() { return shooterX; }
    public double getShooterY() { return shooterY; }
    public double getShooterZ() { return shooterZ; }
    public double getShooterYaw() { return shooterYaw; }
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getTargetZ() { return targetZ; }
    public double getTargetRadius() { return targetRadius; }
    public double getRobotVx() { return robotVx; }
    public double getRobotVy() { return robotVy; }
    /** @deprecated Use {@link #getShotPreference()} instead */
    @Deprecated
    public boolean isPreferHighArc() { return preferHighArc; }
    public boolean isIncludeAirResistance() { return includeAirResistance; }
    
    // New getters
    public ShotPreference getShotPreference() { return shotPreference; }
    public int getMaxCandidates() { return maxCandidates; }
    public double getMinPitchDegrees() { return minPitchDegrees; }
    public double getMaxPitchDegrees() { return maxPitchDegrees; }
    public double getMinVelocityMps() { return minVelocityMps; }
    public double getMaxVelocityMps() { return maxVelocityMps; }
    public double getAngleStepDegrees() { return angleStepDegrees; }
    public double getMinArcHeightMeters() { return minArcHeightMeters; }
    
    @Override
    public String toString() {
        return String.format("Shot from (%.2f, %.2f, %.2f)m yaw=%.1f deg to (%.2f, %.2f, %.2f)m (%.2fm dist, %.2fm high) Robot V=(%.1f, %.1f)",
            shooterX, shooterY, shooterZ, Math.toDegrees(shooterYaw),
            targetX, targetY, targetZ,
            getHorizontalDistanceMeters(), getHeightDifferenceMeters(),
            robotVx, robotVy);
    }
}
