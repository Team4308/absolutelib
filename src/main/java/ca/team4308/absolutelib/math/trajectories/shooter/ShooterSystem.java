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

    private ShotParameters fallbackShot;

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
        this.fallbackShot = new ShotParameters(60.0, 3000, config.rpmToVelocity(3000),
                3.0, ShotParameters.Source.FALLBACK);
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
     * Sets the fallback shot used when the primary calculation fails or is
     * rejected by the safety validator.
     *
     * @param pitchDegrees fallback pitch angle
     * @param rpm          fallback flywheel RPM
     */
    public void setFallbackShot(double pitchDegrees, double rpm) {
        this.fallbackShot = new ShotParameters(pitchDegrees, rpm,
                config.rpmToVelocity(rpm), 0, ShotParameters.Source.FALLBACK);
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
            lastResult = fallbackShot;
            lastSourceDescription = "fallback (bad distance: " + distCheck.reason + ")";
            lastValidation = distCheck;
            return fallbackShot;
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
                lastSourceDescription = "solver";
                break;

            case LOOKUP_WITH_SOLVER_FALLBACK:
                if (lookupTable.hasEntries() && lookupTable.isInRange(distanceMeters)) {
                    base = lookupTable.lookup(distanceMeters);
                    lastSourceDescription = "lookup";
                } else {
                    base = solveWithSolver(distanceMeters, yawToTargetRad);
                    if (!base.valid && lookupTable.hasEntries()) {

                        base = lookupTable.lookup(distanceMeters);
                        lastSourceDescription = "lookup (clamped, solver failed)";
                    } else {
                        lastSourceDescription = "solver (out of table range)";
                    }
                }
                break;

            case SOLVER_WITH_LOOKUP_FALLBACK:
                base = solveWithSolver(distanceMeters, yawToTargetRad);
                if (!base.valid && lookupTable.hasEntries()) {
                    base = lookupTable.lookup(distanceMeters);
                    lastSourceDescription = "lookup (solver fallback)";
                } else {
                    lastSourceDescription = "solver";
                }
                break;

            case BLENDED:
                base = blendResults(distanceMeters, yawToTargetRad);
                lastSourceDescription = "blended";
                break;

            default:
                base = fallbackShot;
                lastSourceDescription = "fallback (unknown mode)";
                break;
        }

        if (!base.valid) {
            lastResult = fallbackShot;
            lastSourceDescription += " → fallback";
            lastValidation = SafetyValidator.ValidationResult.fail("Base calculation failed");
            return fallbackShot;
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

            lastResult = fallbackShot;
            lastSourceDescription += " → fallback (safety)";
            return fallbackShot;
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

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
