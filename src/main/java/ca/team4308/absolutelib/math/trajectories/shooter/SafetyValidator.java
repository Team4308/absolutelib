package ca.team4308.absolutelib.math.trajectories.shooter;

/**
 * Pre-fire safety validator that checks all shot parameters before allowing
 * the shot to proceed.
 * 
 * <p>Verifies:</p>
 * <ul>
 *   <li>Flywheel is at target speed (within abort threshold)</li>
 *   <li>Pitch angle is within safe limits</li>
 *   <li>Distance reading is valid (not NaN, not negative)</li>
 *   <li>RPM is within allowed range</li>
 *   <li>Exit velocity is within safe limits</li>
 *   <li>Shot parameters are valid (not flagged invalid)</li>
 * </ul>
 * 
 * <p>If any check fails, the validator returns a {@link ValidationResult}
 * with the failure reason. The caller can then fall back to the lookup table
 * or a safe preset.</p>
 */
public final class SafetyValidator {

    private final ShooterConfig config;

    public SafetyValidator(ShooterConfig config) {
        this.config = config;
    }

    /**
     * Result of a safety validation check.
     */
    public static final class ValidationResult {
        public final boolean safe;
        public final String reason;

        private ValidationResult(boolean safe, String reason) {
            this.safe = safe;
            this.reason = reason;
        }

        public static ValidationResult ok() {
            return new ValidationResult(true, "");
        }

        public static ValidationResult fail(String reason) {
            return new ValidationResult(false, reason);
        }

        @Override
        public String toString() {
            return safe ? "SAFE" : "UNSAFE: " + reason;
        }
    }

    /**
     * Validates shot parameters before firing. Does NOT check flywheel RPM
     * (use {@link #validateReadyToFire} for the full pre-fire check).
     * 
     * @param shot the shot parameters to validate
     * @return validation result
     */
    public ValidationResult validateShot(ShotParameters shot) {
        if (shot == null) {
            return ValidationResult.fail("Shot parameters are null");
        }
        if (!shot.valid) {
            return ValidationResult.fail("Shot flagged invalid: " + shot.invalidReason);
        }
        if (Double.isNaN(shot.pitchDegrees) || Double.isInfinite(shot.pitchDegrees)) {
            return ValidationResult.fail("Pitch is NaN or infinite");
        }
        if (shot.pitchDegrees < config.getMinPitchDegrees() - 0.1
                || shot.pitchDegrees > config.getMaxPitchDegrees() + 0.1) {
            return ValidationResult.fail(String.format("Pitch %.1f° outside limits [%.1f, %.1f]",
                    shot.pitchDegrees, config.getMinPitchDegrees(), config.getMaxPitchDegrees()));
        }
        if (Double.isNaN(shot.rpm) || shot.rpm < 0) {
            return ValidationResult.fail("RPM is NaN or negative");
        }
        if (shot.rpm > config.getMaxRpm() * 1.05) {
            return ValidationResult.fail(String.format("RPM %.0f exceeds max %.0f",
                    shot.rpm, config.getMaxRpm()));
        }
        if (shot.exitVelocityMps > config.getSafetyMaxExitVelocity()) {
            return ValidationResult.fail(String.format("Exit velocity %.1f m/s exceeds safety max %.1f",
                    shot.exitVelocityMps, config.getSafetyMaxExitVelocity()));
        }
        if (Double.isNaN(shot.distanceMeters) || shot.distanceMeters < 0) {
            return ValidationResult.fail("Distance is NaN or negative");
        }
        if (shot.distanceMeters < config.getMinDistanceMeters()) {
            return ValidationResult.fail(String.format("Distance %.2fm below minimum %.2fm",
                    shot.distanceMeters, config.getMinDistanceMeters()));
        }
        if (shot.distanceMeters > config.getMaxDistanceMeters()) {
            return ValidationResult.fail(String.format("Distance %.2fm above maximum %.2fm",
                    shot.distanceMeters, config.getMaxDistanceMeters()));
        }
        return ValidationResult.ok();
    }

    /**
     * Full pre-fire check: validates shot parameters AND checks that the
     * flywheel has reached target RPM.
     * 
     * @param shot        the shot parameters
     * @param measuredRpm the actual flywheel RPM from sensors
     * @return validation result
     */
    public ValidationResult validateReadyToFire(ShotParameters shot, double measuredRpm) {
        ValidationResult shotResult = validateShot(shot);
        if (!shotResult.safe) {
            return shotResult;
        }
        double deficit = shot.rpm - measuredRpm;
        if (deficit > config.getRpmAbortThreshold()) {
            return ValidationResult.fail(String.format(
                    "Flywheel not ready: %.0f RPM (need %.0f, deficit %.0f)",
                    measuredRpm, shot.rpm, deficit));
        }
        return ValidationResult.ok();
    }

    /**
     * Validates that a distance reading is usable.
     * 
     * @param distanceMeters the measured distance
     * @return validation result
     */
    public ValidationResult validateDistance(double distanceMeters) {
        if (Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters)) {
            return ValidationResult.fail("Distance sensor returned NaN/Infinity");
        }
        if (distanceMeters < 0) {
            return ValidationResult.fail("Negative distance");
        }
        if (distanceMeters < config.getMinDistanceMeters()) {
            return ValidationResult.fail("Too close: " + distanceMeters + "m");
        }
        if (distanceMeters > config.getMaxDistanceMeters()) {
            return ValidationResult.fail("Too far: " + distanceMeters + "m");
        }
        return ValidationResult.ok();
    }
}
