package ca.team4308.absolutelib.math.trajectories.shooter;

import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.TrajectoryResult;
import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;

/**
 * High-level manager for the robot's shooting subsystem.
 * <p>
 * <b>Responsibilities:</b>
 * <ul>
 * <li><b>Strategy Selection:</b> Decides whether to use the Lookup Table (fast,
 * reliable) or the {@link TrajectorySolver} (flexible, physics-based).</li>
 * <li><b>Safety:</b> Prevents unsafe shots using {@link SafetyValidator}.</li>
 * <li><b>Hardware Integration:</b> Manages RPM correction and movement
 * compensation.</li>
 * </ul>
 * <p>
 * <b>Difference from TrajectorySolver:</b>
 * <br> {@link TrajectorySolver} is the <i>Calculator</i> that performs pure
 * physics math to find the necessary angle/velocity. {@code ShooterSystem} is
 * the <i>Manager</i> that uses that calculation to control the robot.
 * <p>
 * <h2>Usage</h2>
 * <pre>{@code
 * ShooterSystem system = new ShooterSystem(config, table, solver);
 * system.setMode(ShotMode.LOOKUP_WITH_SOLVER_FALLBACK);
 *
 * // In periodic:
 * ShotParameters shot = system.calculate(dist, rpm, vx, vy, yaw);
 * if (shot.valid) {
 *     pivot.setPosition(shot.pitchDegrees);
 *     flywheel.setVelocity(shot.rpm);
 * }
 * }</pre>
 */
public final class ShooterSystem {

    private final ShooterConfig config;
    private final ShotLookupTable lookupTable;
    private final TrajectorySolver solver;
    private final RPMCorrector rpmCorrector;
    private final MovementCompensator movementCompensator;
    private final SafetyValidator safetyValidator;

    private ShotMode mode = ShotMode.LOOKUP_WITH_SOLVER_FALLBACK;
    private double blendFactor = 0.5;

    private double manualPitchDegrees = 0;
    private double manualRpm = 0;

    private ShotParameters lastGoodShot = null;

    private ShotParameters lastResult = ShotParameters.invalid("Not yet calculated");
    private SafetyValidator.ValidationResult lastValidation;
    private String lastSourceDescription = "none";
    private TrajectoryResult lastTrajectoryResult;

    private ShotInput solverInput = null;

    /**
     * Creates a new shooter system.
     *
     * @param config shooter configuration
     * @param lookupTable pre-populated lookup table (can be empty for
     * solver-only mode)
     * @param solver trajectory solver instance (can be null for lookup-only
     * mode)
     */
    public ShooterSystem(ShooterConfig config, ShotLookupTable lookupTable, TrajectorySolver solver) {
        this.config = config;
        this.lookupTable = lookupTable;
        this.solver = solver;
        this.rpmCorrector = new RPMCorrector(config);
        this.movementCompensator = new MovementCompensator(config);
        this.safetyValidator = new SafetyValidator(config);
    }

    /**
     * Creates a lookup-table-only shooter system (no solver needed).
     */
    public ShooterSystem(ShooterConfig config, ShotLookupTable lookupTable) {
        this(config, lookupTable, null);
        this.mode = ShotMode.LOOKUP_ONLY;
    }

    /** Sets the active shot calculation mode. */
    public void setMode(ShotMode mode) {
        this.mode = mode;
    }

    /** Returns the active shot calculation mode. */
    public ShotMode getMode() {
        return mode;
    }

    /**
     * Sets the interpolation weight for {@link ShotMode#BLENDED} mode.
     * 0.0 = full lookup, 1.0 = full solver.
     *
     * @param factor blend weight in [0, 1]
     */
    public void setBlendFactor(double factor) {
        this.blendFactor = Math.max(0, Math.min(1, factor));
    }

    /**
     * Sets the manual pitch and RPM for {@link ShotMode#MANUAL} mode.
     *
     * @param pitchDegrees desired pitch angle
     * @param rpm          desired flywheel RPM
     */
    public void setManualOverride(double pitchDegrees, double rpm) {
        this.manualPitchDegrees = pitchDegrees;
        this.manualRpm = rpm;
    }

    /**
     * Updates the input parameters for the solver.
     * <p>
     * Must be called periodically to provide the solver with the latest
     * robot/target state.
     *
     * @param input the populated shot input
     */
    public void setSolverInput(ShotInput input) {
        this.solverInput = input;
    }

    /**
     * Calculates shot parameters for the given conditions.
     *
     * <p>
     * This is the main entry point. Call once per loop iteration.</p>
     *
     * @param distanceMeters horizontal distance to target
     * @param measuredRpm current flywheel RPM from sensors (0 if not available)
     * @param robotVxMps field-relative X velocity (0 if stationary)
     * @param robotVyMps field-relative Y velocity (0 if stationary)
     * @param yawToTargetRad yaw angle from robot to target in radians
     * @return validated shot parameters ready for use
     */
    public ShotParameters calculate(double distanceMeters, double measuredRpm,
            double robotVxMps, double robotVyMps,
            double yawToTargetRad) {
        SafetyValidator.ValidationResult distCheck = safetyValidator.validateDistance(distanceMeters);
        if (!distCheck.safe) {
            ShotParameters invalidShot = ShotParameters.invalid("bad distance: " + distCheck.reason);
            lastResult = invalidShot;
            lastSourceDescription = "invalid (bad distance: " + distCheck.reason + ")";
            lastValidation = distCheck;
            return invalidShot;
        }

        ShotParameters base;
        switch (mode) {
            case MANUAL:
                base = new ShotParameters(manualPitchDegrees, manualRpm,
                        config.rpmToVelocity(manualRpm), distanceMeters, ShotParameters.Source.MANUAL);
                lastSourceDescription = "manual";
                break;

            case LOOKUP_ONLY:
                base = lookupTable.lookup(distanceMeters);
                lastSourceDescription = "lookup";
                break;

            case SOLVER_ONLY:
                base = solveWithSolver(distanceMeters, yawToTargetRad);
                if (base.valid) {
                    lastSourceDescription = "solver";
                } else {
                    base = fallbackToTableOrLastGood(distanceMeters, "solver failed");
                }
                break;

            case LOOKUP_WITH_SOLVER_FALLBACK:
                if (lookupTable.hasEntries() && lookupTable.isInRange(distanceMeters)) {
                    base = lookupTable.lookup(distanceMeters);
                    lastSourceDescription = "lookup";
                } else {
                    base = solveWithSolver(distanceMeters, yawToTargetRad);
                    if (base.valid) {
                        lastSourceDescription = "solver (out of table range)";
                    } else {
                        base = fallbackToTableOrLastGood(distanceMeters, "solver failed, out of table range");
                    }
                }
                break;

            case SOLVER_WITH_LOOKUP_FALLBACK:
                base = solveWithSolver(distanceMeters, yawToTargetRad);
                if (base.valid) {
                    lastSourceDescription = "solver";
                } else if (lookupTable.hasEntries()) {
                    base = lookupTable.lookup(distanceMeters);
                    if (base.valid) {
                        lastSourceDescription = "lookup (solver fallback)";
                    } else {
                        base = fallbackToLastGood("solver and lookup failed");
                    }
                } else {
                    base = fallbackToLastGood("solver failed, no table");
                }
                break;

            case BLENDED:
                base = blendResults(distanceMeters, yawToTargetRad);
                if (base.valid) {
                    lastSourceDescription = "blended";
                } else {
                    base = fallbackToTableOrLastGood(distanceMeters, "blend failed");
                }
                break;

            default:
                base = ShotParameters.invalid("unknown mode");
                lastSourceDescription = "invalid (unknown mode)";
                break;
        }

        if (base.valid && base.source != ShotParameters.Source.FALLBACK
                && base.source != ShotParameters.Source.LAST_KNOWN_GOOD) {
            lastGoodShot = base;
        }

        if (!base.valid) {
            lastResult = base;
            lastSourceDescription += " -> invalid";
            lastValidation = SafetyValidator.ValidationResult.fail("Base calculation failed");
            return base;
        }

        ShotParameters compensated = movementCompensator.compensate(
                base, robotVxMps, robotVyMps, yawToTargetRad);

        ShotParameters corrected;
        if (measuredRpm > 0) {
            corrected = rpmCorrector.correct(compensated, measuredRpm);
        } else {
            corrected = compensated;
        }

        lastValidation = safetyValidator.validateShot(corrected);
        if (!lastValidation.safe) {
            SafetyValidator.ValidationResult baseCheck = safetyValidator.validateShot(base);
            if (baseCheck.safe) {
                lastResult = base;
                lastSourceDescription += " (corrections rejected, using base)";
                return base;
            }

            ShotParameters invalidShot = ShotParameters.invalid("safety rejection");
            lastResult = invalidShot;
            lastSourceDescription += " -> invalid (safety)";
            return invalidShot;
        }

        lastResult = corrected;
        return corrected;
    }

    /**
     * Simplified calculate for stationary robot with no RPM feedback.
     */
    public ShotParameters calculate(double distanceMeters) {
        return calculate(distanceMeters, 0, 0, 0, 0);
    }

    /**
     * Full pre-fire readiness check. Call this before actually shooting.
     *
     * @param measuredRpm current flywheel RPM
     * @return true if all safety checks pass and flywheel is at speed
     */
    public boolean isReadyToFire(double measuredRpm) {
        if (!lastResult.valid) {
            return false;
        }
        return safetyValidator.validateReadyToFire(lastResult, measuredRpm).safe;
    }

    private ShotParameters solveWithSolver(double distanceMeters, double yawToTargetRad) {
        if (solver == null || solverInput == null) {
            return ShotParameters.invalid("Solver not configured");
        }
        try {
            TrajectoryResult result = solver.solve(solverInput);
            lastTrajectoryResult = result;
            if (result.isSuccess()) {
                double pitch = result.getPitchAngleDegrees();
                double rpm = result.getRecommendedRpm();
                double vel = result.getRequiredVelocityMps();
                double yawAdj = result.getYawAdjustmentRadians();
                return new ShotParameters(pitch, rpm, vel, distanceMeters,
                        yawAdj, ShotParameters.Source.SOLVER);
            }
            return ShotParameters.invalid("Solver found no valid trajectory");
        } catch (Exception e) {
            return ShotParameters.invalid("Solver error: " + e.getMessage());
        }
    }

    /**
     * Fallback chain: lookup table (clamped) -> last known good.
     * Used when the primary calculation source fails and a table lookup is reasonable.
     */
    private ShotParameters fallbackToTableOrLastGood(double distanceMeters, String reason) {
        if (lookupTable.hasEntries()) {
            ShotParameters tableResult = lookupTable.lookup(distanceMeters);
            if (tableResult.valid) {
                lastSourceDescription = "lookup (clamped, " + reason + ")";
                return tableResult;
            }
        }
        return fallbackToLastGood(reason);
    }

    /**
     * Fallback chain: last known good -> invalid.
     * Used when neither the primary nor the lookup table produced a valid result.
     */
    private ShotParameters fallbackToLastGood(String reason) {
        if (lastGoodShot != null) {
            lastSourceDescription = "last known good (" + reason + ")";
            return lastGoodShot.withSource(ShotParameters.Source.LAST_KNOWN_GOOD);
        }
        lastSourceDescription = "invalid (" + reason + ")";
        return ShotParameters.invalid(reason);
    }

    private ShotParameters blendResults(double distanceMeters, double yawToTargetRad) {
        ShotParameters lookupResult = lookupTable.hasEntries()
                ? lookupTable.lookup(distanceMeters) : null;
        ShotParameters solverResult = solveWithSolver(distanceMeters, yawToTargetRad);

        if (lookupResult != null && lookupResult.valid && solverResult.valid) {
            double pitch = lerp(lookupResult.pitchDegrees, solverResult.pitchDegrees, blendFactor);
            double rpm = lerp(lookupResult.rpm, solverResult.rpm, blendFactor);
            double vel = lerp(lookupResult.exitVelocityMps, solverResult.exitVelocityMps, blendFactor);
            return new ShotParameters(pitch, rpm, vel, distanceMeters,
                    ShotParameters.Source.BLENDED);
        } else if (lookupResult != null && lookupResult.valid) {
            return lookupResult;
        } else if (solverResult.valid) {
            return solverResult;
        }
        return ShotParameters.invalid("Both lookup and solver failed");
    }

    /** Returns the last computed shot parameters. */
    public ShotParameters getLastResult() {
        return lastResult;
    }

    /** Returns a human-readable description of which source produced the last shot. */
    public String getLastSourceDescription() {
        return lastSourceDescription;
    }

    /**
     * Returns whether the last calculated shot came from a real calculation
     * (solver, lookup, blended) rather than a fallback or last-known-good.
     * Use this to decide whether to show a "shot ready" indicator to the driver.
     */
    public boolean isLastShotFresh() {
        if (lastResult == null || !lastResult.valid) {
            return false;
        }
        return lastResult.source != ShotParameters.Source.FALLBACK
                && lastResult.source != ShotParameters.Source.LAST_KNOWN_GOOD;
    }

    /**
     * Returns the last known good shot, or null if no successful calculation
     * has been performed yet.
     */
    public ShotParameters getLastGoodShot() {
        return lastGoodShot;
    }

    /**
     * Clears the cached last-known-good shot. Useful when the robot
     * re-localizes or the target changes significantly.
     */
    public void clearLastGoodShot() {
        lastGoodShot = null;
    }

    /** Returns the last safety validation result, or null if not yet computed. */
    public SafetyValidator.ValidationResult getLastValidation() {
        return lastValidation;
    }

    /** Returns the shooter configuration. */
    public ShooterConfig getConfig() {
        return config;
    }

    /** Returns the lookup table. */
    public ShotLookupTable getLookupTable() {
        return lookupTable;
    }

    /**
     * Returns the RPM corrector for direct access.
     */
    public RPMCorrector getRpmCorrector() {
        return rpmCorrector;
    }

    /**
     * Returns the movement compensator for direct access.
     */
    public MovementCompensator getMovementCompensator() {
        return movementCompensator;
    }

    /**
     * Returns the safety validator for direct access.
     */
    public SafetyValidator getSafetyValidator() {
        return safetyValidator;
    }

    /**
     * Returns the last trajectory result from the solver, or null if the solver
     * has not been called yet.
     *
     * @return the last trajectory result, or null
     */
    public TrajectoryResult getLastTrajectoryResult() {
        return lastTrajectoryResult;
    }

    /**
     * Returns the underlying trajectory solver, or null if not configured.
     */
    public TrajectorySolver getSolver() {
        return solver;
    }

    /**
     * Returns the horizontal distance to the target from the last solver input,
     * or 0 if no input has been set. Useful for dashboard telemetry.
     */
    public double getDistanceToTarget() {
        if (solverInput == null) {
            return 0;
        }
        return solverInput.getHorizontalDistanceMeters();
    }

    /**
     * Returns the last calculated shot's source as a human-readable string.
     * Shorthand for {@code getLastResult().source.name()}.
     */
    public String getLastSourceName() {
        return lastResult != null ? lastResult.source.name() : "NONE";
    }

    /**
     * Unified telemetry information for the shooter system.
     * Use this to get all relevant status data in one call for dashboarding.
     */
    public static class ShooterTelemetry {
        public final ShotMode mode;
        public final ShotParameters.Source source;
        public final String sourceDetail;
        public final double distanceMeters;
        public final double targetRpm;
        public final double targetPitchDegrees;
        public final boolean isValid;
        public final boolean isReady;
        public final SafetyValidator.ValidationResult safetyResult;

        public ShooterTelemetry(ShotMode mode, ShotParameters result, String sourceDetail, 
                double distance, boolean isReady, SafetyValidator.ValidationResult safety) {
            this.mode = mode;
            this.isValid = result != null && result.valid;
            
            if (result != null) {
                this.source = result.source;
                this.targetRpm = result.rpm;
                this.targetPitchDegrees = result.pitchDegrees;
            } else {
                this.source = ShotParameters.Source.FALLBACK;
                this.targetRpm = 0;
                this.targetPitchDegrees = 0;
            }
            
            this.sourceDetail = sourceDetail != null ? sourceDetail : "none";
            this.distanceMeters = distance;
            this.isReady = isReady;
            this.safetyResult = safety;
        }

        @Override
        public String toString() {
            return String.format("Mode: %s | Source: %s (%s) | Dist: %.2fm | RPM: %.0f | Pitch: %.1f° | Valid: %b | Ready: %b",
                    mode, source, sourceDetail, distanceMeters, targetRpm, targetPitchDegrees, isValid, isReady);
        }
    }

    /**
     * Returns a snapshot of the system's current telemetry.
     * 
     * @param measuredRpm current flywheel RPM from sensors
     * @return snapshots of the shooter state
     */
    public ShooterTelemetry getSystemTelemetry(double measuredRpm) {
        return new ShooterTelemetry(
                mode, 
                lastResult, 
                lastSourceDescription, 
                getDistanceToTarget(),
                isReadyToFire(measuredRpm),
                lastValidation
        );
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
