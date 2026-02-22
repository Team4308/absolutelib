package ca.team4308.absolutelib.math.trajectories.shooter;

/**
 * Compensates shot parameters for robot movement so shots stay accurate
 * without stopping.
 * 
 * <p>When the robot is moving toward or away from the target, the ball
 * inherits the chassis velocity. This compensator iteratively adjusts the
 * effective target position and tweaks RPM/pitch so the ball still lands
 * in the goal.</p>
 * 
 * <p>Inspired by the iterative approach in the houndutil
 * {@code ShootOnTheFlyCalculator} and the hammerheads5000 {@code TurretCalculator},
 * but simplified to a linear correction term for speed and stability.</p>
 * 
 * <h2>Usage</h2>
 * <pre>{@code
 * MovementCompensator comp = new MovementCompensator(config);
 * ShotParameters adjusted = comp.compensate(baseShotParams, vxMps, vyMps, yawToTargetRad);
 * }</pre>
 */

public final class MovementCompensator {

    private final ShooterConfig config;

    /**
     * Creates a compensator using the specified shooter configuration.
     *
     * @param config shooter configuration providing compensation gain,
     *               iteration count, and pitch/RPM limits
     */
    public MovementCompensator(ShooterConfig config) {
        this.config = config;
    }

    /**
     * Adjusts shot parameters to compensate for robot chassis velocity.
     * 
     * <p>Uses an iterative approach: estimates time of flight, projects the
     * virtual target position, re-estimates TOF, and repeats for the configured
     * number of iterations.</p>
     * 
     * <p>When the base shot comes from the solver (which already adjusts the
     * target position for movement), only the yaw lead is computed to avoid
     * double-compensation of the radial component.</p>
     * 
     * @param base              base shot parameters (from lookup or solver)
     * @param robotVxMps        field-relative robot velocity X (m/s)
     * @param robotVyMps        field-relative robot velocity Y (m/s)
     * @param yawToTargetRad    yaw angle from robot to target (radians)
     * @return movement-compensated shot parameters
     */

    public ShotParameters compensate(ShotParameters base, double robotVxMps, double robotVyMps,
                                     double yawToTargetRad) {
        if (!base.valid) {
            return base;
        }

        double speed = Math.hypot(robotVxMps, robotVyMps);
        if (speed < 0.05) {
            return base;
        }

        double exitVelocity = base.exitVelocityMps;
        if (!Double.isFinite(exitVelocity) || exitVelocity <= 0) {
            return base;
        }
        double horizontalSpeed = exitVelocity * Math.cos(Math.toRadians(base.pitchDegrees));
        if (!Double.isFinite(horizontalSpeed) || horizontalSpeed < 0.1) horizontalSpeed = 0.1;

        double tof = base.distanceMeters / horizontalSpeed;

        double lateralVelocity = -robotVxMps * Math.sin(yawToTargetRad)
                                + robotVyMps * Math.cos(yawToTargetRad);

        double yawLead = Math.atan2(-lateralVelocity * tof, base.distanceMeters);

        if (base.source == ShotParameters.Source.SOLVER) {
            double combinedYaw = base.yawAdjustmentRadians + yawLead;
            return new ShotParameters(base.pitchDegrees, base.rpm, base.exitVelocityMps,
                    base.distanceMeters, combinedYaw, ShotParameters.Source.MOVING_COMPENSATED);
        }

        double gain = config.getMovingCompensationGain();
        int iterations = config.getMovingIterations();

        double radialVelocity = robotVxMps * Math.cos(yawToTargetRad)
                              + robotVyMps * Math.sin(yawToTargetRad);

        double effectiveDistance = base.distanceMeters;

        for (int i = 0; i < iterations; i++) {
            effectiveDistance = base.distanceMeters - radialVelocity * tof * gain;
            if (effectiveDistance < 0.3) effectiveDistance = 0.3;

            horizontalSpeed = exitVelocity * Math.cos(Math.toRadians(base.pitchDegrees));
            if (horizontalSpeed < 0.1) horizontalSpeed = 0.1;
            double newTof = effectiveDistance / horizontalSpeed;

            if (Math.abs(newTof - tof) < 0.005) break;
            tof = newTof;
        }

        yawLead = Math.atan2(-lateralVelocity * tof, effectiveDistance);

        double rpmAdjustment = -radialVelocity * (base.rpm / exitVelocity) * gain;
        double newRpm = base.rpm + rpmAdjustment;
        newRpm = clamp(newRpm, config.getMinRpm(), config.getMaxRpm());

        double distanceDelta = effectiveDistance - base.distanceMeters;
        double pitchAdjust = distanceDelta * 0.5;
        double newPitch = base.pitchDegrees + pitchAdjust;
        newPitch = clamp(newPitch, config.getMinPitchDegrees(), config.getMaxPitchDegrees());

        double newExitVelocity = config.rpmToVelocity(newRpm);

        return new ShotParameters(newPitch, newRpm, newExitVelocity,
                effectiveDistance, yawLead, ShotParameters.Source.MOVING_COMPENSATED);
    }

    /**
     * Calculates the yaw lead angle to account for lateral robot movement.
     * The turret or drivetrain should apply this offset.
     * 
     * @param robotVxMps     field-relative X velocity (m/s)
     * @param robotVyMps     field-relative Y velocity (m/s)
     * @param yawToTargetRad current yaw to target (radians)
     * @param tofSeconds     estimated time of flight (seconds)
     * @param distanceMeters distance to target (meters)
     * @return yaw lead offset in radians (add to current yaw)
     */

    public double calculateYawLead(double robotVxMps, double robotVyMps,
                                   double yawToTargetRad, double tofSeconds,
                                   double distanceMeters) {
        double lateralVelocity = -robotVxMps * Math.sin(yawToTargetRad)
                                + robotVyMps * Math.cos(yawToTargetRad);

        return Math.atan2(-lateralVelocity * tofSeconds, distanceMeters);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
