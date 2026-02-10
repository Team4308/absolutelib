package ca.team4308.absolutelib.math.trajectories.shooter;

/**
 * RPM feedback corrector that adjusts shot pitch when the flywheel is slightly
 * below target speed.
 * 
 * <p>When the measured RPM is below the target RPM by more than the configured
 * threshold, the corrector increases the pitch slightly to compensate. If the
 * deficit is above the abort threshold the shot is flagged as unsafe.</p>
 * 
 * <p>This corrects for battery sag, wheel slip, motor heating, and gamepiece
 * compression changes — all without a full trajectory re-solve.</p>
 * 
 * <h3>Usage</h3>
 * <pre>{@code
 * RPMCorrector corrector = new RPMCorrector(config);
 * ShotParameters corrected = corrector.correct(baseShotParams, measuredRpm);
 * }</pre>
 */
public final class RPMCorrector {

    private final ShooterConfig config;

    public RPMCorrector(ShooterConfig config) {
        this.config = config;
    }

    /**
     * Applies RPM feedback correction to a base shot.
     * 
     * @param base        the base shot parameters (from lookup or solver)
     * @param measuredRpm the actual flywheel RPM from sensors
     * @return corrected shot parameters, or an invalid shot if the deficit is too large
     */
    public ShotParameters correct(ShotParameters base, double measuredRpm) {
        if (!base.valid) {
            return base;
        }

        double deficit = base.rpm - measuredRpm;

        if (deficit <= 0) {
            return base;
        }

        if (deficit > config.getRpmAbortThreshold()) {
            return ShotParameters.invalid(
                    String.format("RPM deficit too large: %.0f (target %.0f, actual %.0f)",
                            deficit, base.rpm, measuredRpm));
        }

        if (deficit < config.getRpmFeedbackThreshold()) {
            return base;
        }

        double pitchIncrease = deficit * config.getPitchCorrectionPerRpmDeficit();
        double newPitch = base.pitchDegrees + pitchIncrease;

        newPitch = clamp(newPitch, config.getMinPitchDegrees(), config.getMaxPitchDegrees());

        return base.withPitchDegrees(newPitch).withSource(ShotParameters.Source.RPM_FEEDBACK);
    }

    /**
     * Returns true if the measured RPM is close enough to the target to fire.
     * 
     * @param targetRpm   the desired flywheel RPM
     * @param measuredRpm the actual flywheel RPM
     * @return true if the deficit is within the abort threshold
     */
    public boolean isReadyToFire(double targetRpm, double measuredRpm) {
        return (targetRpm - measuredRpm) < config.getRpmAbortThreshold();
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
