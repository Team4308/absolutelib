package ca.team4308.absolutelib.math.trajectories.shooter;

import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.TrajectoryResult;
import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;

/**
 * Hybrid shooter system that combines a lookup table, physics solver, RPM
 * feedback, and movement compensation into a single easy-to-use API.
 * 
 * <h3>Design Goals</h3>
 * <ul>
 *   <li>Reliable in matches — lookup table as primary source</li>
 *   <li>Easy to tune on the practice field — just add table entries</li>
 *   <li>Safe if math or sensors fail — safety validator + fallback</li>
 *   <li>Stable under battery sag and wear — RPM feedback correction</li>
 *   <li>Fast to compute, low memory — small fixed arrays, simple math</li>
 * </ul>
 * 
 * <h3>Usage</h3>
 * <pre>{@code
 * ShooterConfig config = ShooterConfig.defaults2026();
 * ShotLookupTable table = new ShotLookupTable()
 *     .addEntry(1.5, 75.0, 2000)
 *     .addEntry(3.0, 60.0, 3200)
 *     .addEntry(5.0, 50.0, 4200);
 * TrajectorySolver solver = TrajectorySolver.forGame2026();
 * 
 * ShooterSystem system = new ShooterSystem(config, table, solver);
 * system.setMode(ShotMode.LOOKUP_WITH_SOLVER_FALLBACK);
 * 
 * // In periodic():
 * ShotParameters shot = system.calculate(distanceM, measuredRpm, vx, vy, yawRad);
 * if (shot.valid) {
 *     pivot.setAngle(shot.pitchDegrees);
 *     flywheel.setRPM(shot.rpm);
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

    private ShotInput.Builder solverInputTemplate = null;

    /**
     * Creates a new shooter system.
     * 
     * @param config      shooter configuration
     * @param lookupTable pre-populated lookup table (can be empty for solver-only mode)
     * @param solver      trajectory solver instance (can be null for lookup-only mode)
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

    /** Sets the operating mode. */
    public void setMode(ShotMode mode) {
        this.mode = mode;
    }

    /** Returns the current operating mode. */
    public ShotMode getMode() {
        return mode;
    }

    /** Sets the blend factor for {@link ShotMode#BLENDED} mode (0 = all lookup, 1 = all solver). */
    public void setBlendFactor(double factor) {
        this.blendFactor = Math.max(0, Math.min(1, factor));
    }

    /** Sets manual override values for {@link ShotMode#MANUAL} mode. */
    public void setManualOverride(double pitchDegrees, double rpm) {
        this.manualPitchDegrees = pitchDegrees;
        this.manualRpm = rpm;
    }

    /** Sets the fallback safe shot used when all calculations fail. */
    public void setFallbackShot(double pitchDegrees, double rpm) {
        this.fallbackShot = new ShotParameters(pitchDegrees, rpm,
                config.rpmToVelocity(rpm), 0, ShotParameters.Source.FALLBACK);
    }

    /**
     * Sets a template for solver inputs. This provides the solver with
     * shooter/target positions and other parameters. Must be set for solver modes.
     */
    public void setSolverInputTemplate(ShotInput.Builder template) {
        this.solverInputTemplate = template;
    }

    /**
     * Calculates shot parameters for the given conditions.
     * 
     * <p>This is the main entry point. Call once per loop iteration.</p>
     * 
     * @param distanceMeters   horizontal distance to target
     * @param measuredRpm      current flywheel RPM from sensors (0 if not available)
     * @param robotVxMps       field-relative X velocity (0 if stationary)
     * @param robotVyMps       field-relative Y velocity (0 if stationary)
     * @param yawToTargetRad   yaw angle from robot to target in radians
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
        if (!lastResult.valid) return false;
        return safetyValidator.validateReadyToFire(lastResult, measuredRpm).safe;
    }

    private ShotParameters solveWithSolver(double distanceMeters, double yawToTargetRad) {
        if (solver == null || solverInputTemplate == null) {
            return ShotParameters.invalid("Solver not configured");
        }
        try {
            ShotInput input = solverInputTemplate.build();
            TrajectoryResult result = solver.solve(input);
            if (result.isSuccess()) {
                double pitch = result.getPitchAngleDegrees();
                double rpm = result.getRecommendedRpm();
                double vel = result.getRequiredVelocityMps();
                return new ShotParameters(pitch, rpm, vel, distanceMeters,
                        ShotParameters.Source.SOLVER);
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
                    ShotParameters.Source.LOOKUP_TABLE);
        } else if (lookupResult != null && lookupResult.valid) {
            return lookupResult;
        } else if (solverResult.valid) {
            return solverResult;
        }
        return ShotParameters.invalid("Both lookup and solver failed");
    }

    /** Returns the last calculated shot parameters. */
    public ShotParameters getLastResult() { return lastResult; }

    /** Returns a human-readable description of the last source used. */
    public String getLastSourceDescription() { return lastSourceDescription; }

    /** Returns the last safety validation result. */
    public SafetyValidator.ValidationResult getLastValidation() { return lastValidation; }

    /** Returns the underlying config. */
    public ShooterConfig getConfig() { return config; }

    /** Returns the lookup table for adding entries at runtime. */
    public ShotLookupTable getLookupTable() { return lookupTable; }

    /** Returns the RPM corrector for direct access. */
    public RPMCorrector getRpmCorrector() { return rpmCorrector; }

    /** Returns the movement compensator for direct access. */
    public MovementCompensator getMovementCompensator() { return movementCompensator; }

    /** Returns the safety validator for direct access. */
    public SafetyValidator getSafetyValidator() { return safetyValidator; }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
