package ca.team4308.absolutelib.math.trajectories.shooter;

/**
 * Immutable shot parameters: angle, RPM, and exit velocity.
 * This is the output of any shot calculation method (lookup table, solver, etc.).
 * 
 * <p>All angles are in degrees, velocities in m/s, RPM in rotations per minute.</p>
 */
public final class ShotParameters {

    /** Pitch angle in degrees (0 = horizontal, 90 = straight up). */
    public final double pitchDegrees;

    /** Flywheel RPM required for this shot. */
    public final double rpm;

    /** Exit velocity in m/s. */
    public final double exitVelocityMps;

    /** Distance to target in meters (for reference). */
    public final double distanceMeters;

    /** Source that produced these parameters. */
    public final Source source;

    /** Whether this shot is considered valid and safe to fire. */
    public final boolean valid;

    /** Human-readable reason if shot is invalid. */
    public final String invalidReason;

    /**
     * Identifies where these shot parameters came from.
     */
    public enum Source {
        /** Interpolated from the lookup table. */
        LOOKUP_TABLE,
        /** Computed by the physics trajectory solver. */
        SOLVER,
        /** RPM feedback corrected the original solution. */
        RPM_FEEDBACK,
        /** Movement-compensated solution. */
        MOVING_COMPENSATED,
        /** Fallback / safe preset. */
        FALLBACK,
        /** Manual override from driver. */
        MANUAL
    }

    public ShotParameters(double pitchDegrees, double rpm, double exitVelocityMps,
                          double distanceMeters, Source source) {
        this.pitchDegrees = pitchDegrees;
        this.rpm = rpm;
        this.exitVelocityMps = exitVelocityMps;
        this.distanceMeters = distanceMeters;
        this.source = source;
        this.valid = true;
        this.invalidReason = "";
    }

    private ShotParameters(double pitchDegrees, double rpm, double exitVelocityMps,
                           double distanceMeters, Source source,
                           boolean valid, String invalidReason) {
        this.pitchDegrees = pitchDegrees;
        this.rpm = rpm;
        this.exitVelocityMps = exitVelocityMps;
        this.distanceMeters = distanceMeters;
        this.source = source;
        this.valid = valid;
        this.invalidReason = invalidReason;
    }

    /**
     * Creates an invalid shot with the given reason.
     */
    public static ShotParameters invalid(String reason) {
        return new ShotParameters(0, 0, 0, 0, Source.FALLBACK, false, reason);
    }

    /**
     * Returns a copy with a different source tag.
     */
    public ShotParameters withSource(Source newSource) {
        return new ShotParameters(pitchDegrees, rpm, exitVelocityMps,
                distanceMeters, newSource, valid, invalidReason);
    }

    /**
     * Returns a copy with adjusted pitch.
     */
    public ShotParameters withPitchDegrees(double newPitch) {
        return new ShotParameters(newPitch, rpm, exitVelocityMps,
                distanceMeters, source, valid, invalidReason);
    }

    /**
     * Returns a copy with adjusted RPM.
     */
    public ShotParameters withRpm(double newRpm) {
        return new ShotParameters(pitchDegrees, newRpm, exitVelocityMps,
                distanceMeters, source, valid, invalidReason);
    }

    /**
     * Returns the pitch angle in radians.
     */
    public double getPitchRadians() {
        return Math.toRadians(pitchDegrees);
    }

    @Override
    public String toString() {
        if (!valid) {
            return String.format("INVALID: %s", invalidReason);
        }
        return String.format("%.1f° %.0fRPM %.1fm/s @ %.2fm [%s]",
                pitchDegrees, rpm, exitVelocityMps, distanceMeters, source);
    }
}
