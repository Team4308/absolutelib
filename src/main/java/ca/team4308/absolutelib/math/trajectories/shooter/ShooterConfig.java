package ca.team4308.absolutelib.math.trajectories.shooter;

/**
 * Configuration for the hybrid shooter system.
 * 
 * <p>Immutable after construction. Use the builder to create instances.</p>
 * 
 * <h2>Usage</h2>
 * <pre>{@code
 * ShooterConfig config = ShooterConfig.builder()
 *     .pitchLimits(30.0, 80.0)
 *     .rpmLimits(500, 6000)
 *     .rpmToVelocityFactor(0.00532)
 *     .maxDistanceMeters(12.0)
 *     .rpmFeedbackThreshold(50.0)
 *     .rpmAbortThreshold(500.0)
 *     .movingCompensationGain(1.0)
 *     .build();
 * }</pre>
 */
public final class ShooterConfig {

    private final double minPitchDegrees;
    private final double maxPitchDegrees;

    private final double minRpm;
    private final double maxRpm;

    private final double rpmToVelocityFactor;

    private final double minDistanceMeters;
    private final double maxDistanceMeters;

    private final double rpmFeedbackThreshold;
    private final double rpmAbortThreshold;
    private final double pitchCorrectionPerRpmDeficit;

    private final double movingCompensationGain;
    private final int movingIterations;

    private final double safetyMaxExitVelocity;

    private ShooterConfig(Builder b) {
        this.minPitchDegrees = b.minPitchDegrees;
        this.maxPitchDegrees = b.maxPitchDegrees;
        this.minRpm = b.minRpm;
        this.maxRpm = b.maxRpm;
        this.rpmToVelocityFactor = b.rpmToVelocityFactor;
        this.minDistanceMeters = b.minDistanceMeters;
        this.maxDistanceMeters = b.maxDistanceMeters;
        this.rpmFeedbackThreshold = b.rpmFeedbackThreshold;
        this.rpmAbortThreshold = b.rpmAbortThreshold;
        this.pitchCorrectionPerRpmDeficit = b.pitchCorrectionPerRpmDeficit;
        this.movingCompensationGain = b.movingCompensationGain;
        this.movingIterations = b.movingIterations;
        this.safetyMaxExitVelocity = b.safetyMaxExitVelocity;
    }

    public double getMinPitchDegrees() { return minPitchDegrees; }
    public double getMaxPitchDegrees() { return maxPitchDegrees; }
    public double getMinRpm() { return minRpm; }
    public double getMaxRpm() { return maxRpm; }
    public double getRpmToVelocityFactor() { return rpmToVelocityFactor; }
    public double getMinDistanceMeters() { return minDistanceMeters; }
    public double getMaxDistanceMeters() { return maxDistanceMeters; }
    public double getRpmFeedbackThreshold() { return rpmFeedbackThreshold; }
    public double getRpmAbortThreshold() { return rpmAbortThreshold; }
    public double getPitchCorrectionPerRpmDeficit() { return pitchCorrectionPerRpmDeficit; }
    public double getMovingCompensationGain() { return movingCompensationGain; }
    public int getMovingIterations() { return movingIterations; }
    public double getSafetyMaxExitVelocity() { return safetyMaxExitVelocity; }

    /** Converts RPM to approximate exit velocity using the configured factor. */
    public double rpmToVelocity(double rpm) {
        return rpm * rpmToVelocityFactor;
    }

    /** Converts exit velocity back to approximate RPM using the configured factor. */
    public double velocityToRpm(double velocityMps) {
        return rpmToVelocityFactor > 0 ? velocityMps / rpmToVelocityFactor : 0;
    }

    /** Creates a new builder with sensible defaults. */
    public static Builder builder() {
        return new Builder();
    }

    /** Returns a pre-configured config suitable for a typical 2026 game shooter. */
    public static ShooterConfig defaults2026() {
        return builder()
                .pitchLimits(40.0, 82.0)
                .rpmLimits(500, 6000)
                .rpmToVelocityFactor(0.00532)
                .distanceLimits(0.5, 12.0)
                .rpmFeedbackThreshold(50.0)
                .rpmAbortThreshold(500.0)
                .pitchCorrectionPerRpmDeficit(0.005)
                .movingCompensationGain(1.0)
                .movingIterations(5)
                .safetyMaxExitVelocity(30.0)
                .build();
    }

    public static final class Builder {
        private double minPitchDegrees = 30.0;
        private double maxPitchDegrees = 82.0;
        private double minRpm = 0;
        private double maxRpm = 6000;
        private double rpmToVelocityFactor = 0.00532;
        private double minDistanceMeters = 0.5;
        private double maxDistanceMeters = 15.0;
        private double rpmFeedbackThreshold = 50.0;
        private double rpmAbortThreshold = 500.0;
        private double pitchCorrectionPerRpmDeficit = 0.005;
        private double movingCompensationGain = 1.0;
        private int movingIterations = 5;
        private double safetyMaxExitVelocity = 30.0;

        /** Set min and max pitch in degrees. */
        public Builder pitchLimits(double minDeg, double maxDeg) {
            this.minPitchDegrees = minDeg;
            this.maxPitchDegrees = maxDeg;
            return this;
        }

        /** Set min and max flywheel RPM. */
        public Builder rpmLimits(double min, double max) {
            this.minRpm = min;
            this.maxRpm = max;
            return this;
        }

        /** Factor to convert RPM → m/s exit velocity (default 0.00532 for 4-inch wheel). */
        public Builder rpmToVelocityFactor(double factor) {
            this.rpmToVelocityFactor = factor;
            return this;
        }

        /** Set min and max distance in meters. */
        public Builder distanceLimits(double minM, double maxM) {
            this.minDistanceMeters = minM;
            this.maxDistanceMeters = maxM;
            return this;
        }

        /** RPM deficit below which feedback correction kicks in. */
        public Builder rpmFeedbackThreshold(double threshold) {
            this.rpmFeedbackThreshold = threshold;
            return this;
        }

        /** RPM deficit above which shot is aborted instead of corrected. */
        public Builder rpmAbortThreshold(double threshold) {
            this.rpmAbortThreshold = threshold;
            return this;
        }

        /** Degrees of pitch correction per 1 RPM of deficit (linear correction). */
        public Builder pitchCorrectionPerRpmDeficit(double correction) {
            this.pitchCorrectionPerRpmDeficit = correction;
            return this;
        }

        /** Gain applied to robot velocity during moving compensation (1.0 = full). */
        public Builder movingCompensationGain(double gain) {
            this.movingCompensationGain = gain;
            return this;
        }

        /** Number of iterations for the iterative moving shot solver. */
        public Builder movingIterations(int iterations) {
            this.movingIterations = iterations;
            return this;
        }

        /** Maximum allowed exit velocity for safety validation. */
        public Builder safetyMaxExitVelocity(double maxMps) {
            this.safetyMaxExitVelocity = maxMps;
            return this;
        }

        public ShooterConfig build() {
            return new ShooterConfig(this);
        }
    }

    @Override
    public String toString() {
        return String.format("ShooterConfig[pitch=%.1f–%.1f°, rpm=%.0f–%.0f, dist=%.1f–%.1fm]",
                minPitchDegrees, maxPitchDegrees, minRpm, maxRpm, minDistanceMeters, maxDistanceMeters);
    }
}
