package ca.team4308.absolutelib.math.trajectories;

import java.util.List;

import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelGenerator;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelSimulator;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;
import ca.team4308.absolutelib.math.trajectories.physics.AirResistance;
import ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion;
import ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap;

/**
 * Trajectory solver for FRC shooting. Handles projectile physics, flywheel
 * selection.
 *
 * <p>
 * This solver acts as the "Brain" of the shooter, translating desired target hits into 
 * physical actuator setpoints. It encapsulates a high-fidelity RK4 integrator and 
 * multiple search strategies to balance accuracy with computation budget.
 * </p>
 * 
 * <h2>Configuration Hierarchy</h2>
 * <p>The system is split into two distinct configuration layers:</p>
 * <ul>
 *   <li><b>SolverConfig (This Class):</b> Governs the <i>Physics &amp; Math</i> logic. 
 *       Includes simulation timesteps, search tolerances, and solve modes. Change these for performance or accuracy tuning.</li>
 *   <li><b>ShooterConfig (ShooterSystem):</b> Governs the <i>Mechanical/Electronic</i> hardware.
 *       Includes physical hard-stops, gear ratios, and conversion factors. Change these to match your robot's build.</li>
 * </ul>
 */
public class TrajectorySolver {

    /**
     * Strategy for finding a trajectory.
     */
    public enum SolveMode {

        /**
         * Direct algebraic solver. Computes the required velocity and pitch by solving
         * the system of equations for a given entry angle or peak clearance.
         * Best for long-range shots where the parabolic arc is well-behaved.
         */
        CONSTRAINT,
        /**
         * Discrete search across the entire pitch range. Simulates multiple trajectories
         * and picks the one with the smallest miss distance.
         * Best for complex scenarios with high drag or vertical obstacles.
         */
        SWEEP,
        /**
         * Fast-lookup mode. Interpolates between precomputed states for near-instant execution.
         */
        MAP,
        /**
         * Binary Search (Bisection) solving. Much faster than SWEEP (uses ~5
         * iterations instead of ~30) while keeping full physics simulation
         * accuracy.
         */
        BISECTION,
        /**
         * Hybrid mode for roboRIO: uses lookup table to seed a narrow, high-precision
         * bisection search.
         */
        HYBRID
    }

    /**
     * Internal data structure for tracking real-time calibration samples.
     * Each sample records an actual trajectory execution result that can be
     * used to improve the RPM-to-velocity conversion factor.
     */
    private static class CalibrationSample {
        final double distanceMeters;
        final double pitchDegrees;
        final double rpmUsed;
        final double expectedDistance;
        final double actualDistance;
        final double distanceError; // actual - expected
        final long timestampMs;

        CalibrationSample(double distanceMeters, double pitchDegrees, double rpmUsed,
                         double expectedDistance, double actualDistance) {
            this.distanceMeters = distanceMeters;
            this.pitchDegrees = pitchDegrees;
            this.rpmUsed = rpmUsed;
            this.expectedDistance = expectedDistance;
            this.actualDistance = actualDistance;
            this.distanceError = actualDistance - expectedDistance;
            this.timestampMs = System.currentTimeMillis();
        }
    }

    /**
     * Configuration for the trajectory math engine.
     * 
     * <p>Use this to tune the "how" of the search: speed vs. precision. 
     * Constants like {@link #simulationTimeStep} and {@link #angleTolerance} 
     * live here.</p>
     */
    public static class SolverConfig {

        private final double minPitchDegrees;
        private final double maxPitchDegrees;
        private final double minRpm;
        private final double maxRpm;
        private final boolean useParallel;

        private final double rpmTolerance;
        private final double angleTolerance;

        private final FlywheelGenerator.GenerationParams flywheelGenParams;

        private final double crtRpmResolution;
        private final double crtAngleResolution;
        private final int crtControlLoopMs;
        private final int crtEncoderTicks;

        /**
         * Multiplier for target radius when checking hits.
         */
        private final double hoopToleranceMultiplier;

        /**
         * Sweep angle step size (degrees).
         */
        private final double sweepStepDegrees;

        /**
         * Number of iterations used to refine the initial velocity estimate after
         * drag simulation reveals a miss. Higher values closer the "Velocity Gap"
         * created by air resistance.
         */
        private final int velocityRefineIterations;

        /**
         * The temporal resolution (seconds) of the physics integrator. 
         * A step of 0.02 matches the RIO loop frequency, while 0.005 provides
         * extreme precision for long-range drag modeling.
         */
        private final double simulationTimeStep;

        /**
         * Fast simulation timestep (seconds) for search/sweep passes. Larger =
         * faster but less accurate. Default is 5x the simulation timestep.
         */
        private final double fastSimulationTimeStep;

        private SolverConfig(Builder builder) {
            this.minPitchDegrees = builder.minPitchDegrees;
            this.maxPitchDegrees = builder.maxPitchDegrees;
            this.minRpm = builder.minRpm;
            this.maxRpm = builder.maxRpm;
            this.rpmTolerance = builder.rpmTolerance;
            this.angleTolerance = builder.angleTolerance;
            this.flywheelGenParams = builder.flywheelGenParams;
            this.crtRpmResolution = builder.crtRpmResolution;
            this.crtAngleResolution = builder.crtAngleResolution;
            this.crtControlLoopMs = builder.crtControlLoopMs;
            this.crtEncoderTicks = builder.crtEncoderTicks;
            this.hoopToleranceMultiplier = builder.hoopToleranceMultiplier;
            this.sweepStepDegrees = builder.sweepStepDegrees;
            this.velocityRefineIterations = builder.velocityRefineIterations;
            this.simulationTimeStep = builder.simulationTimeStep;
            this.fastSimulationTimeStep = builder.fastSimulationTimeStep;
            this.useParallel = builder.useParallel;
        }

        public double getMinPitchDegrees() {
            return minPitchDegrees;
        }

        public double getMaxPitchDegrees() {
            return maxPitchDegrees;
        }

        public double getMinRpm() {
            return minRpm;
        }

        public double getMaxRpm() {
            return maxRpm;
        }

        public double getRpmTolerance() {
            return rpmTolerance;
        }

        public double getAngleTolerance() {
            return angleTolerance;
        }

        public FlywheelGenerator.GenerationParams getFlywheelGenParams() {
            return flywheelGenParams;
        }

        public double getCrtRpmResolution() {
            return crtRpmResolution;
        }

        public double getCrtAngleResolution() {
            return crtAngleResolution;
        }

        public int getCrtControlLoopMs() {
            return crtControlLoopMs;
        }

        public int getCrtEncoderTicks() {
            return crtEncoderTicks;
        }

        public double getHoopToleranceMultiplier() {
            return hoopToleranceMultiplier;
        }

        public double getSweepStepDegrees() {
            return sweepStepDegrees;
        }

        public int getVelocityRefineIterations() {
            return velocityRefineIterations;
        }

        public double getSimulationTimeStep() {
            return simulationTimeStep;
        }

        public double getFastSimulationTimeStep() {
            return fastSimulationTimeStep;
        }

        public boolean useParallel() {
            return useParallel;
        }

        /**
         * Creates a new solver config builder with default values.
         */
        public static Builder builder() {
            return new Builder();
        }

        /**
         * Creates a builder pre-populated with this config's values.
         */
        public Builder toBuilder() {
            return new Builder()
                    .minPitchDegrees(minPitchDegrees)
                    .maxPitchDegrees(maxPitchDegrees)
                    .minRpm(minRpm)
                    .maxRpm(maxRpm)
                    .rpmTolerance(rpmTolerance)
                    .angleTolerance(angleTolerance)
                    .flywheelGenParams(flywheelGenParams)
                    .crtRpmResolution(crtRpmResolution)
                    .crtAngleResolution(crtAngleResolution)
                    .crtControlLoopMs(crtControlLoopMs)
                    .crtEncoderTicks(crtEncoderTicks)
                    .hoopToleranceMultiplier(hoopToleranceMultiplier)
                    .sweepStepDegrees(sweepStepDegrees)
                    .velocityRefineIterations(velocityRefineIterations)
                    .simulationTimeStep(simulationTimeStep)
                    .fastSimulationTimeStep(fastSimulationTimeStep)
                    .useParallel(useParallel);
        }

        /**
         * Builder for fluent construction of {@link SolverConfig}. All values
         * have sensible defaults for typical FRC use.
         */
        public static class Builder {

            private double minPitchDegrees = 0;
            private double maxPitchDegrees = 90;
            private double minRpm = 0;
            // Cap RPM for typical FRC shooters so solver does not request unrealistically high speeds.
            // This is also used during precompute generation to ensure lookup tables stay within
            // mechanism limits.
            private double maxRpm = 6000;

            private double rpmTolerance = 100;
            private double angleTolerance = 1.0;

            private FlywheelGenerator.GenerationParams flywheelGenParams
                    = FlywheelGenerator.GenerationParams.defaultParams();

            private double crtRpmResolution = 1.0;
            private double crtAngleResolution = 0.1;
            private int crtControlLoopMs = 20;
            private int crtEncoderTicks = 4096;
            private double hoopToleranceMultiplier = 1.0;
            private double sweepStepDegrees = 0.5;
            private int velocityRefineIterations = 8;
            private double simulationTimeStep = ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.DEFAULT_TIME_STEP;
            private double fastSimulationTimeStep = ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.DEFAULT_TIME_STEP * 5.0;
            private boolean useParallel = false;

            /**
             * Sets the minimum launch pitch angle in degrees (default 0).
             */
            public Builder minPitchDegrees(double val) {
                this.minPitchDegrees = val;
                return this;
            }

            /**
             * Sets the maximum launch pitch angle in degrees (default 90).
             */
            public Builder maxPitchDegrees(double val) {
                this.maxPitchDegrees = val;
                return this;
            }

            /**
             * Sets the minimum flywheel RPM to consider (default 0).
             */
            public Builder minRpm(double val) {
                this.minRpm = val;
                return this;
            }

            /**
             * Sets the maximum flywheel RPM to consider (default 10,000).
             */
            public Builder maxRpm(double val) {
                this.maxRpm = val;
                return this;
            }

            /**
             * Sets the RPM convergence tolerance (default 100).
             */
            public Builder rpmTolerance(double val) {
                this.rpmTolerance = val;
                return this;
            }

            /**
             * Sets the angle convergence tolerance in degrees (default 1.0).
             */
            public Builder angleTolerance(double val) {
                this.angleTolerance = val;
                return this;
            }

            /**
             * Sets the flywheel generation parameters for CRT sweep.
             */
            public Builder flywheelGenParams(FlywheelGenerator.GenerationParams val) {
                this.flywheelGenParams = val;
                return this;
            }

            /**
             * Sets the CRT RPM resolution in RPM (default 1.0).
             */
            public Builder crtRpmResolution(double val) {
                this.crtRpmResolution = val;
                return this;
            }

            /**
             * Sets the CRT angle resolution in degrees (default 0.1).
             */
            public Builder crtAngleResolution(double val) {
                this.crtAngleResolution = val;
                return this;
            }

            /**
             * Sets the CRT control loop period in milliseconds (default 20).
             */
            public Builder crtControlLoopMs(int val) {
                this.crtControlLoopMs = val;
                return this;
            }

            /**
             * Sets the CRT encoder ticks per revolution (default 4096).
             */
            public Builder crtEncoderTicks(int val) {
                this.crtEncoderTicks = val;
                return this;
            }

            /**
             * Sets the hoop tolerance multiplier for target acceptance (default
             * 1.0).
             */
            public Builder hoopToleranceMultiplier(double val) {
                this.hoopToleranceMultiplier = val;
                return this;
            }

            /**
             * Sets the sweep angle step size in degrees (default 0.5).
             */
            public Builder sweepStepDegrees(double val) {
                this.sweepStepDegrees = val;
                return this;
            }

            /**
             * Sets the number of binary-search iterations for velocity
             * refinement (default 8).
             */
            public Builder velocityRefineIterations(int val) {
                this.velocityRefineIterations = val;
                return this;
            }

            /**
             * Sets the simulation timestep in seconds (default 0.001).
             */
            public Builder simulationTimeStep(double val) {
                this.simulationTimeStep = val;
                return this;
            }

            /**
             * Sets the fast simulation timestep in seconds (default 0.005).
             */
            public Builder fastSimulationTimeStep(double val) {
                this.fastSimulationTimeStep = val;
                return this;
            }

            /**
             * Sets whether to use parallel execution for certain solver tasks
             * (default false).
             */
            public Builder useParallel(boolean val) {
                this.useParallel = val;
                return this;
            }

            /**
             * Builds the solver config.
             */
            public SolverConfig build() {
                return new SolverConfig(this);
            }
        }

        /**
         * Creates a builder from a DTO.
         */
        public static SolverConfig.Builder fromDTO(ca.team4308.absolutelib.math.trajectories.network.TrajectoryConfigDTO dto) {
            return new SolverConfig.Builder()
                .minPitchDegrees(dto.shooterPitchMin) // Using shooter limits for solver too
                .maxPitchDegrees(dto.shooterPitchMax)
                .minRpm(dto.shooterRpmMin)
                .maxRpm(dto.shooterRpmMax)
                .rpmTolerance(dto.solverRpmTolerance)
                .angleTolerance(dto.solverAngleTolerance)
                .hoopToleranceMultiplier(dto.solverHoopMultiplier)
                .sweepStepDegrees(dto.solverSweepStep)
                .velocityRefineIterations(dto.solverVelRefineIters)
                .simulationTimeStep(dto.solverSimStep)
                .fastSimulationTimeStep(dto.solverFastSimStep)
                .useParallel(dto.solverUseParallel);
        }

        /**
         * Returns a config with all default values.
         */
        public static SolverConfig defaults() {
            return builder().build();
        }

        /**
         * Returns a preset tuned for high accuracy: finer angle/RPM tolerances
         * and detailed flywheel generation.
         */
        public static SolverConfig highAccuracy() {
            return builder()
                    .angleTolerance(0.5)
                    .rpmTolerance(50)
                    .flywheelGenParams(FlywheelGenerator.GenerationParams.detailed())
                    .crtAngleResolution(0.05)
                    .build();
        }

        /**
         * Returns a preset tuned for speed: coarser tolerances and quick-scan
         * flywheel generation.
         */
        public static SolverConfig quickSolve() {
            return builder()
                    .angleTolerance(2.0)
                    .rpmTolerance(200)
                    .flywheelGenParams(FlywheelGenerator.GenerationParams.quickScan())
                    .sweepStepDegrees(2.0)
                    .velocityRefineIterations(4)
                    .simulationTimeStep(0.002)
                    .fastSimulationTimeStep(0.01)
                    .build();
        }

        /**
         * Returns a preset optimized for roboRIO hardware. Balances solve speed
         * against accuracy for real-time use on the ARM Cortex-A9. Uses a
         * coarser sweep, fewer refinement iterations, and a larger simulation
         * timestep to reduce CPU load.
         */
        public static SolverConfig roboRIO() {
            return builder()
                    .angleTolerance(1.5)
                    .rpmTolerance(150)
                    .flywheelGenParams(FlywheelGenerator.GenerationParams.quickScan())
                    .sweepStepDegrees(1.5)
                    .velocityRefineIterations(5)
                    .simulationTimeStep(0.002)
                    .fastSimulationTimeStep(0.008)
                    .build();
        }

        /**
         * Returns a preset optimized for powerful co-processors (e.g. Orange Pi 5,
         * Beelink Mini PC). Uses high-fidelity simulations, fine-grained
         * sweeping, and multi-threaded execution.
         */
        public static SolverConfig coProcessor() {
            return builder()
                    .angleTolerance(0.2)
                    .rpmTolerance(25)
                    .flywheelGenParams(FlywheelGenerator.GenerationParams.detailed())
                    .sweepStepDegrees(0.25)
                    .velocityRefineIterations(15)
                    .simulationTimeStep(0.001)
                    .fastSimulationTimeStep(0.004)
                    .useParallel(true)
                    .build();
        }

        /**
         * Returns a high-performance preset for the roboRIO. Uses the bisection
         * solver for speed while keeping a relatively fine simulation timestep.
         */
        public static SolverConfig roboRIOPerformance() {
            return builder()
                    .angleTolerance(0.8)
                    .rpmTolerance(80)
                    .flywheelGenParams(FlywheelGenerator.GenerationParams.detailed())
                    .sweepStepDegrees(1.0)
                    .velocityRefineIterations(8)
                    .simulationTimeStep(0.002)
                    .fastSimulationTimeStep(0.008)
                    .build();
        }
    }

    private GamePiece gamePiece;
    private SolverConfig config;
    private final ProjectileMotion projectileMotion;
    private final FlywheelGenerator flywheelGenerator;

    private final ca.team4308.absolutelib.math.trajectories.impl.InterpolatingDoubleTreeMap tuningPitchMap = new ca.team4308.absolutelib.math.trajectories.impl.InterpolatingDoubleTreeMap();
    private final ca.team4308.absolutelib.math.trajectories.impl.InterpolatingDoubleTreeMap tuningRpmMap = new ca.team4308.absolutelib.math.trajectories.impl.InterpolatingDoubleTreeMap();

    private boolean hasTuningPitch = false;
    private boolean hasTuningRpm = false;

    /**
     * Empirical shot map built from real measured robot data.
     * When set, the solver uses interpolated RPM and pitch from this map
     * instead of deriving RPM from flywheel physics simulation.
     */
    private EmpiricalShotMap empiricalMap = null;

    /**
     * Sets the empirical shot map for calibration-based solving.
     * When an empirical map is loaded and the query distance is within its range,
     * {@link #solve(ShotInput)} will return interpolated RPM and pitch from
     * the map instead of computing RPM through the flywheel physics model.
     *
     * <p>This is the recommended approach for competition robots that have been
     * calibrated with real shot data.
     *
     * @param map the empirical map, or null to disable
     * @see EmpiricalShotMap
     */
    public void setEmpiricalMap(EmpiricalShotMap map) {
        this.empiricalMap = map;
        // Feed empirical data points into the internal tuning maps so that
        // scoring functions (computeSweepQualityScore, etc.) bias angle
        // selection toward measured values.
        if (map != null && map.hasData()) {
            for (EmpiricalShotMap.DataPoint pt : map.getDataPoints()) {
                tuningPitchMap.put(pt.distanceMeters, pt.pitchDegrees);
                tuningRpmMap.put(pt.distanceMeters, pt.rpm);
            }
            hasTuningPitch = true;
            hasTuningRpm = true;
        }
    }

    /**
     * Returns the currently loaded empirical map, or null if none.
     */
    public EmpiricalShotMap getEmpiricalMap() {
        return empiricalMap;
    }

    /**
     * Provide empirical tuning points so the solver biases toward specific RPM/Pitch combinations at known distances.
     * <p>If an {@link EmpiricalShotMap} has been set via {@link #setEmpiricalMap(EmpiricalShotMap)},
     * this method also adds the point to that map. Otherwise, points are stored in internal
     * interpolation maps for scoring bias.
     */
    public void addTuningPoint(double distanceMeters, double pitchDegrees, double rpm) {
        if (pitchDegrees >= 0) {
            tuningPitchMap.put(distanceMeters, pitchDegrees);
            hasTuningPitch = true;
        }
        if (rpm >= 0) {
            tuningRpmMap.put(distanceMeters, rpm);
            hasTuningRpm = true;
        }
        // Also route to the empirical map if one is set
        if (empiricalMap != null && pitchDegrees >= 0 && rpm >= 0) {
            empiricalMap.addPoint(distanceMeters, pitchDegrees, rpm);
        }
    }

    /**
     * Imports all points from a {@link ca.team4308.absolutelib.math.trajectories.shooter.ShotLookupTable} as tuning points.
     * 
     * @param table the table to import tuning points from
     */
    public void addTuningPoint(ca.team4308.absolutelib.math.trajectories.shooter.ShotLookupTable table) {
        if (table == null || !table.hasEntries()) {
            return;
        }
        for (Double distance : table.getPitchMap().keySet()) {
            Double pitch = table.getPitchMap().get(distance);
            Double rpm = table.getRpmMap().get(distance);
            if (pitch != null && rpm != null) {
                addTuningPoint(distance, pitch, rpm);
            }
        }
    }

    private FlywheelConfig cachedFlywheel;
    private boolean debugEnabled = false;
    private SolveMode solveMode = SolveMode.CONSTRAINT;

    // Last successful solution state used to dampen close-range jitter between
    // successive solves.
    private double previousPitchAngleRadians = Double.NaN;
    private double previousYawAdjustmentRadians = Double.NaN;
    private double previousRpm = Double.NaN;

    // Real-time calibration tracking: learns RPM-to-velocity factor from actual shot results
    private double adaptiveRpmToVelocityFactor = 0.01532; // Initial fallback from DefaultShotTable
    private final java.util.List<CalibrationSample> calibrationSamples = new java.util.ArrayList<>();
    private static final int MAX_CALIBRATION_SAMPLES = 100; // Keep rolling window of recent samples
    private static final double CALIBRATION_SMOOTHING_ALPHA = 0.15; // Exponential moving average weight
    private static final double MIN_CALIBRATION_SAMPLES_FOR_CONFIDENCE = 3;

    /**
     * Sets the solve strategy.
     */
    public void setSolveMode(SolveMode mode) {
        this.solveMode = (mode != null) ? mode : SolveMode.CONSTRAINT;
    }

    /**
     * Returns the current solve strategy.
     */
    public SolveMode getSolveMode() {
        return solveMode;
    }

    /**
     * Enables or disables debug recording. Adds overhead; disable for
     * competition.
     */
    public void setDebugEnabled(boolean enabled) {
        this.debugEnabled = enabled;
    }

    /**
     * Returns whether debug mode is currently enabled.
     */
    public boolean isDebugEnabled() {
        return debugEnabled;
    }

    /**
     * Updates the solver configuration at runtime.
     * 
     * @param newConfig the new configuration to apply
     */
    public synchronized void updateConfig(SolverConfig newConfig) {
        if (newConfig == null) return;
        this.config = newConfig;
        // The projectile motion and flywheel generator are initialized in the constructor
        // but they don't depend on config values that can change at runtime here
        // (timesteps are used during solve() calls).
    }

    /**
     * Records the actual result of a trajectory to refine the RPM-to-velocity calibration factor.
     * 
     * <p>Call this method after executing a shot to provide feedback about actual vs. calculated
     * performance. The solver will use this data to adapt its RPM-to-velocity conversion factor
     * toward the true hardware parameters, eliminating assumptions about wheel size or gear ratio.</p>
     * 
     * <p>Example usage in your shooter subsystem:</p>
     * <pre>
     * // After a shot is executed...
     * double calculatedDistance = 4.5; // From trajectory calculation
     * double actualDistance = 4.52;    // From vision/odometry
     * solver.recordTrajectoryResult(calculatedDistance, actualDistance, 
     *                                pitchDeg, rpmUsed);
     * </pre>
     * 
     * @param expectedDistance Calculated horizontal distance at which ball should land (meters)
     * @param actualDistance Measured horizontal distance where ball actually landed (meters)
     * @param pitchDegrees Pitch angle used in the shot (degrees)
     * @param rpmUsed Wheel RPM that was commanded (RPM)
     */
    public synchronized void recordTrajectoryResult(double expectedDistance, double actualDistance,
                                                    double pitchDegrees, double rpmUsed) {
        // Ignore invalid samples
        if (Double.isNaN(expectedDistance) || Double.isNaN(actualDistance) || 
            Double.isNaN(pitchDegrees) || Double.isNaN(rpmUsed)) {
            return;
        }
        
        if (rpmUsed <= 0 || expectedDistance <= 0 || actualDistance <= 0) {
            return;
        }
        
        // Create and store the sample
        CalibrationSample sample = new CalibrationSample(expectedDistance, pitchDegrees, 
                                                         rpmUsed, expectedDistance, actualDistance);
        calibrationSamples.add(sample);
        
        // Keep rolling window of recent samples
        if (calibrationSamples.size() > MAX_CALIBRATION_SAMPLES) {
            calibrationSamples.remove(0);
        }
        
        // Update adaptive RPM factor using exponential moving average
        updateAdaptiveRpmFactor(sample);
    }

    /**
     * Internal method to update the adaptive RPM-to-velocity factor based on a new sample.
     */
    private void updateAdaptiveRpmFactor(CalibrationSample sample) {
        if (sample.rpmUsed <= 0 || sample.distanceError == 0) {
            return;
        }
        
        // Basic adjustment: if we fell short, we need more velocity, so increase the factor
        // If we overshot, we need less velocity, so decrease the factor
        // Adjustment magnitude is proportional to the error
        double errorRatio = sample.distanceError / sample.expectedDistance;
        double adjustment = adaptiveRpmToVelocityFactor * errorRatio * 0.01; // 1% adjustment per 1% error
        
        // Apply exponential moving average to smooth out noise
        double newFactor = adaptiveRpmToVelocityFactor + (adjustment * CALIBRATION_SMOOTHING_ALPHA);
        
        // Sanity bounds: factor should stay reasonable (0.005 to 0.025)
        // Based on typical wheel sizes and gear ratios
        newFactor = Math.max(0.005, Math.min(0.025, newFactor));
        
        adaptiveRpmToVelocityFactor = newFactor;
    }

    /**
     * Gets the current adaptive RPM-to-velocity conversion factor.
     * 
     * <p>This factor is initially set to 0.01532 from DefaultShotTable, but adapts based on
     * actual trajectory feedback. Returns the adapted value if sufficient calibration data
     * exists, otherwise returns the fallback.</p>
     * 
     * @return RPM-to-velocity factor (m/s per RPM)
     */
    public synchronized double getAdaptiveRpmFactor() {
        if (calibrationSamples.size() >= MIN_CALIBRATION_SAMPLES_FOR_CONFIDENCE) {
            return adaptiveRpmToVelocityFactor;
        }
        // Fallback to DefaultShotTable calibration if not enough samples yet
        return 0.01532;
    }

    /**
     * Returns the number of calibration samples recorded so far.
     * 
     * @return Number of trajectory results used for calibration
     */
    public synchronized int getCalibrationSampleCount() {
        return calibrationSamples.size();
    }

    /**
     * Resets all calibration data and returns the adaptive factor to the default value.
     * Use this if you swap hardware or want to start fresh.
     */
    public synchronized void resetCalibration() {
        calibrationSamples.clear();
        adaptiveRpmToVelocityFactor = 0.01532;
    }

    /**
     * Min pitch range (deg) after forcing high arc. If too narrow, skip the
     * force.
     */
    private static final double MIN_FORCED_ARC_RANGE_DEG = 15.0;

    /**
     * Distance (m) at which full drag compensation kicks in. Linearly
     * interpolated below this.
     */
    private static final double DRAG_COMP_FULL_RANGE_METERS = 8.0;

    /** Hard minimum allowed wheel RPM for close-range shots. */
    private static final double CLOSE_RANGE_MIN_RPM = 1700.0;

    /**
     * Vacuum launch velocity for a given pitch, distance, and height
     * difference.
     *
     * @return velocity in m/s, or NaN if the angle can't reach
     */
    static double calculateRequiredVelocityForPitch(double distance, double heightDiff, double pitchRad) {
        double cosTheta = Math.cos(pitchRad);
        double tanTheta = Math.tan(pitchRad);
        double denominator = distance * tanTheta - heightDiff;
        if (denominator <= 0.001) {
            return Double.NaN;

        }
        double g = ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.GRAVITY;
        return Math.sqrt(g * distance * distance / (2.0 * cosTheta * cosTheta * denominator));
    }

    /**
     * Two-constraint parabolic solver. Models trajectory as y = ax^2 + bx,
     * enforces (1) ball hits target center and (2) ball clears rim edge. Solves
     * for a and b, then derives pitch and velocity.
     *
     * @param d horizontal distance to target (m)
     * @param h height difference (m)
     * @param r target opening radius (m)
     * @param c rim clearance height (m)
     * @return {pitchRadians, velocityMps} or null
     */
    static double[] computeConstraintSolution(double d, double h, double r, double c) {

        if (d <= r || r <= 0 || d < 0.3) {
            return null;
        }

        double g = ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.GRAVITY;

        double denom = d * r * (d - r);
        if (Math.abs(denom) < 1e-10) {
            return null;
        }

        double a = -(h * r + c * d) / denom;

        double b = (h - a * d * d) / d;

        double theta = Math.atan(b);
        if (theta <= 0) {
            return null;
        }

        double cosTheta = Math.cos(theta);
        double v0Sq = -g / (2.0 * a * cosTheta * cosTheta);
        if (v0Sq <= 0) {
            return null;
        }

        return new double[]{theta, Math.sqrt(v0Sq)};
    }

    /**
     * Distance-scaled drag compensation. Linearly ramps from 1.0 at close range
     * to full dragCompensationMultiplier at long range.
     */
    static double calculateDragCompensation(double distance) {
        double closeRange = SolverConstants.getCloseRangeThresholdMeters();
        double fullDragComp = SolverConstants.getDragCompensationMultiplier();

        if (distance <= closeRange) {
            return 1.0;
        }
        if (distance >= DRAG_COMP_FULL_RANGE_METERS) {
            return fullDragComp;
        }

        double t = (distance - closeRange) / (DRAG_COMP_FULL_RANGE_METERS - closeRange);
        return 1.0 + t * (fullDragComp - 1.0);
    }

    /**
     * Binary-searches velocity to land the ball through the rim plane within
     * the target opening. Corrects for drag overshoot. Uses simulateFast() for
     * search iterations since only hit/miss metrics are needed.
     *
     * @return a hit result, or null if nothing in range works
     */
    private ProjectileMotion.TrajectoryResult refineVelocityForHit(
            GamePiece gp,
            double shooterX, double shooterY, double shooterZ,
            double pitchRad, double yawRad, double spinRpm,
            double robotVx, double robotVy,
            double targetX, double targetY, double targetZ, double targetRadius,
            double initialVelocity) {

        double vLow = initialVelocity * 0.50; // 
        double vHigh = initialVelocity * 1.05; // 

        ProjectileMotion.TrajectoryResult bestHit = null;
        int iterations = config.getVelocityRefineIterations();

        for (int i = 0; i < iterations; i++) {
            double vMid = (vLow + vHigh) / 2.0;
            ProjectileMotion.TrajectoryResult result = projectileMotion.simulateFast(
                    gp, shooterX, shooterY, shooterZ,
                    vMid, pitchRad, yawRad, spinRpm,
                    robotVx, robotVy,
                    targetX, targetY, targetZ, targetRadius);

            if (result.hitTarget) {
                bestHit = result;
                vHigh = vMid;
            } else if (result.entryAngleDegrees >= 0) {
                vHigh = vMid;
            } else {
                vLow = vMid;
            }
        }

        if (bestHit != null) {
            double bestV = (vLow + vHigh) / 2.0;
            bestHit = projectileMotion.simulate(
                    gp, shooterX, shooterY, shooterZ,
                    bestV, pitchRad, yawRad, spinRpm,
                    robotVx, robotVy,
                    targetX, targetY, targetZ, targetRadius);
        }

        return bestHit;
    }

    /**
     * Checks if a trajectory collides with any obstacle, respecting grace
     * distance and the opening exemption for descending balls.
     */
    private static boolean trajectoryCollides(ProjectileMotion.TrajectoryResult trajSim,
            ShotInput input, double shooterX, double shooterY) {
        return trajectoryCollidesInternal(trajSim, input, shooterX, shooterY, false);
    }

    /**
     * Checks collision with optional verbose logging for diagnostics.
     */
    static boolean trajectoryCollidesInternal(ProjectileMotion.TrajectoryResult trajSim,
            ShotInput input, double shooterX, double shooterY, boolean verbose) {
        if (!input.isCollisionCheckEnabled() || trajSim.trajectory.length == 0) {
            return false;
        }

        double graceDistance = SolverConstants.getCollisionGraceDistanceMeters();
        double graceDist2 = graceDistance * graceDistance;

        for (ObstacleConfig obstacle : input.getObstacles()) {
            for (ProjectileMotion.TrajectoryState state : trajSim.trajectory) {
                if (state == null) {
                    break;
                }
                double sdx = state.x - shooterX;
                double sdy = state.y - shooterY;
                if (sdx * sdx + sdy * sdy < graceDist2) {
                    continue;
                }

                if (state.vz < 0 && obstacle.isWithinOpening(state.x, state.y)) {
                    continue;
                }

                if (obstacle.checkCollision(state.x, state.y, state.z)) {
                    if (verbose) {
                        double distFromCenter = Math.sqrt(
                                Math.pow(state.x - obstacle.getCenterX(), 2)
                                + Math.pow(state.y - obstacle.getCenterY(), 2));
                        System.out.printf("    COLLISION at pt: (%.3f, %.3f, %.3f) vz=%.2f "
                                + "distFromCenter=%.3f opening=%.3f wallH=%.2f totalH=%.2f%n",
                                state.x, state.y, state.z, state.vz,
                                distFromCenter, obstacle.getOpeningDiameter() / 2.0,
                                obstacle.getWallHeight(), obstacle.getTotalHeight());
                    }
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Checks if the ball flies over the target without descending into it.
     * Exempt if the ball is descending and horizontally close enough to enter.
     * Uses a tighter tolerance (60% of target radius) to reject borderline
     * shots that barely clear the hoop edge but would likely fly over in
     * practice.
     */
    private static boolean isFlyover(ProjectileMotion.TrajectoryState[] trajectory,
            double targetX, double targetY, double targetZ, double targetRadius) {
        if (trajectory == null || trajectory.length == 0) {
            return false;
        }

        double bestHorizDist2 = Double.MAX_VALUE;
        double heightAtBestHoriz = 0;
        double vzAtBestHoriz = 0;

        for (ProjectileMotion.TrajectoryState st : trajectory) {
            double dx = st.x - targetX;
            double dy = st.y - targetY;
            double hd2 = dx * dx + dy * dy;
            if (hd2 < bestHorizDist2) {
                bestHorizDist2 = hd2;
                heightAtBestHoriz = st.z;
                vzAtBestHoriz = st.vz;
            }
        }

        double bestHorizDist = Math.sqrt(bestHorizDist2);

        if (vzAtBestHoriz < 0 && bestHorizDist <= targetRadius * 0.6) {
            double hSpeed = 0;
            for (ProjectileMotion.TrajectoryState st : trajectory) {
                double dx = st.x - targetX;
                double dy = st.y - targetY;
                double hd2 = dx * dx + dy * dy;
                if (Math.abs(hd2 - bestHorizDist2) < 1e-6) {
                    hSpeed = Math.sqrt(st.vx * st.vx + st.vy * st.vy);
                    break;
                }
            }
            double entryAngle = Math.toDegrees(Math.atan2(Math.abs(vzAtBestHoriz), Math.max(hSpeed, 1e-6)));
            if (entryAngle >= SolverConstants.getMinEntryAngleDegrees()) {
                return false;
            }
        }

        return heightAtBestHoriz > targetZ;
    }

    /**
     * Constraint-based core: solves the two-constraint system, validates with
     * RK4 + velocity refinement. Returns {pitch, targetX, targetY, tof} or
     * null.
     */
    private double[] solveConstraintCore(
            ShotInput input, FlywheelSimulator flywheelSimForPitch, GamePiece gp,
            double effectiveTargetX, double effectiveTargetY, double estimatedTof,
            double distance, double heightDiff, double dragComp,
            double effectiveMinPitch, double effectiveMaxPitch,
            double requiredClearance, boolean moving, SolveDebugInfo debugInfo) {

        double rimClearance = SolverConstants.getRimClearanceMeters();
        double currentClearance = rimClearance;

        double bestPitch = Double.NaN;
        ProjectileMotion.TrajectoryResult bestTraj = null;
    double bestScore = -Double.MAX_VALUE;
        double itX = effectiveTargetX, itY = effectiveTargetY;

        while (currentClearance <= rimClearance + 3.0) {
            double dx = effectiveTargetX - input.getShooterX();
            double dy = effectiveTargetY - input.getShooterY();
        
            double iterDistance = Math.sqrt(dx * dx + dy * dy);
            double requiredYaw = Math.atan2(dy, dx);

            double[] csol = computeConstraintSolution(
                    iterDistance, heightDiff, input.getTargetRadius(), currentClearance);
            if (csol == null) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(0,
                            SolveDebugInfo.RejectionReason.MISSED_TARGET,
                            Double.MAX_VALUE, 0, 0, false, new ProjectileMotion.TrajectoryState[0]);
                }
                currentClearance += 0.25;
                continue;
            }

            double pitchRad = csol[0];
            double vacuumV = csol[1];

            if (pitchRad < Math.toRadians(effectiveMinPitch)) {
                pitchRad = Math.toRadians(effectiveMinPitch);
                vacuumV = calculateRequiredVelocityForPitch(iterDistance, heightDiff, pitchRad);
                if (Double.isNaN(vacuumV) || vacuumV <= 0) {
                    currentClearance += 0.25;
                    continue;
                }
            } else if (pitchRad > Math.toRadians(effectiveMaxPitch)) {
                pitchRad = Math.toRadians(effectiveMaxPitch);
                vacuumV = calculateRequiredVelocityForPitch(iterDistance, heightDiff, pitchRad);
                if (Double.isNaN(vacuumV) || vacuumV <= 0) {
                    currentClearance += 0.25;
                    continue;
                }
            }

            double pitchDeg = Math.toDegrees(pitchRad);
            double pitchDragComp = 1.0 + (dragComp - 1.0) * Math.cos(pitchRad);
            double targetV = vacuumV * pitchDragComp;

            FlywheelSimulator.SimulationResult pitchFw
                    = flywheelSimForPitch.simulateForVelocity(targetV);
            if (!pitchFw.isAchievable) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg,
                            SolveDebugInfo.RejectionReason.ARC_TOO_LOW,
                            Double.MAX_VALUE, 0, 0, false, new ProjectileMotion.TrajectoryState[0]);
                }
                currentClearance += 0.25;
                continue;
            }

            double actualVelocity = pitchFw.exitVelocityMps;

            ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                    gp,
                    input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                    actualVelocity, pitchRad, requiredYaw,
                    pitchFw.ballSpinRpm,
                    input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                    effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                    input.getTargetRadius()
            );

            if (!trajSim.hitTarget && trajSim.maxHeight > input.getTargetZ()) {
                ProjectileMotion.TrajectoryResult refined = refineVelocityForHit(
                        gp,
                        input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                        pitchRad, requiredYaw, pitchFw.ballSpinRpm,
                        input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                        effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                        input.getTargetRadius(), actualVelocity);
                if (refined != null) {
                    trajSim = refined;
                    if (refined.trajectory.length > 0) {
                        ProjectileMotion.TrajectoryState s0 = refined.trajectory[0];
                        actualVelocity = Math.sqrt(s0.vx * s0.vx + s0.vy * s0.vy + s0.vz * s0.vz);
                    }
                    pitchFw = flywheelSimForPitch.simulateForVelocity(actualVelocity);
                    if (!pitchFw.isAchievable) {
                        currentClearance += 0.25;
                        continue;
                    }
                }
            }

            if (trajSim.flightTime > 0) {
                estimatedTof = trajSim.flightTime;
            }

            if (requiredClearance > 0 && trajSim.maxHeight < requiredClearance) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.CLEARANCE_TOO_LOW,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                currentClearance += 0.25;
                continue;
            }

            boolean finalValid = true;
            double minArcHeight = input.getMinArcHeightMeters();
            if (minArcHeight > 0 && trajSim.maxHeight < input.getTargetZ() + minArcHeight) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.ARC_TOO_LOW,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                currentClearance += 0.25;
                finalValid = false;
            }

            if (finalValid) {
                double hoopTolerance = input.getTargetRadius() * config.getHoopToleranceMultiplier();
                boolean hitsTarget = trajSim.hitTarget
                        || (trajSim.descendingAtClosest && trajSim.closestApproach <= hoopTolerance
                        && trajSim.entryAngleDegrees >= SolverConstants.getMinEntryAngleDegrees());
                if (!hitsTarget) {
                    if (debugInfo != null) {
                        debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.MISSED_TARGET,
                                trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                    }
                    currentClearance += 0.25;
                    finalValid = false;
                }
            }
            if (finalValid && isFlyover(trajSim.trajectory, effectiveTargetX, effectiveTargetY,
                    input.getTargetZ(), input.getTargetRadius())) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.FLYOVER,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                currentClearance += 0.25;
                finalValid = false;
            }

            if (finalValid) {
                double missDistance = (trajSim.horizontalDistAtCrossing >= 0)
                        ? trajSim.horizontalDistAtCrossing : trajSim.closestApproach;
                double score = computeTrajectoryCandidateScore(input, pitchDeg, trajSim,
                        missDistance, pitchFw.requiredWheelRpm, iterDistance, requiredYaw);
                if (score > bestScore) {
                    bestScore = score;
                    bestPitch = pitchRad;
                    bestTraj = trajSim;
                    if (debugInfo != null) {
                        debugInfo.recordAccepted(pitchDeg, missDistance,
                                trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime,
                                trajSim.hitTarget, trajSim.trajectory);
                    }
                }
            }

            currentClearance += 0.25;
        }

        if (bestTraj == null) {
            return null;
        }

        return new double[]{bestPitch, effectiveTargetX, effectiveTargetY, bestTraj.flightTime, 0.0};
    }

    /**
     * Sweep core: tests every pitch at configurable degree steps using fast
     * simulation, picks highest quality score. Collision and flyover checks are
     * deferred to the final full-accuracy validation pass. Returns {pitch,
     * targetX, targetY, tof} or null.
     */
    private double[] solveSweepCore(
            ShotInput input, FlywheelSimulator flywheelSimForPitch, GamePiece gp,
            double effectiveTargetX, double effectiveTargetY, double estimatedTof,
            double distance, double heightDiff, double dragComp,
            double effectiveMinPitch, double effectiveMaxPitch,
            double requiredClearance, boolean moving, SolveDebugInfo debugInfo) {

        double sweepStep = config.getSweepStepDegrees();
        int steps = (int) Math.ceil((effectiveMaxPitch - effectiveMinPitch) / sweepStep) + 1;

        java.util.stream.IntStream stepStream = java.util.stream.IntStream.range(0, steps);
        if (config.useParallel()) {
            stepStream = stepStream.parallel();
        }

        final double finalEstimatedTof = estimatedTof;
        SweepCandidate best = stepStream.mapToObj(i -> {
            double pitchDeg = effectiveMinPitch + i * sweepStep;
            double pitchRad = Math.toRadians(pitchDeg);

            double iterTargetX = effectiveTargetX;
            double iterTargetY = effectiveTargetY;


            // TODO: Remove this is handled by the main robot method
            double dx = iterTargetX - input.getShooterX();
            double dy = iterTargetY - input.getShooterY();
            double iterDistance = Math.sqrt(dx * dx + dy * dy);
            double requiredYaw = Math.atan2(dy, dx);

            double vacuumV = calculateRequiredVelocityForPitch(iterDistance, heightDiff, pitchRad);
            if (Double.isNaN(vacuumV) || vacuumV <= 0) {
                return null;
            }

            double vacuumHoriz = vacuumV * Math.cos(pitchRad);
            double compensatedHoriz = vacuumHoriz * dragComp;
            double vVert = compensatedHoriz * Math.tan(pitchRad);
            double targetV = Math.sqrt(compensatedHoriz * compensatedHoriz + vVert * vVert);

            FlywheelSimulator.SimulationResult pitchFw = flywheelSimForPitch.simulateForVelocity(targetV);
            double dynamicMaxRpm = config.getMaxRpm();
            if (pitchFw.requiredWheelRpm > dynamicMaxRpm) {
                return null;
            }
            if (!pitchFw.isAchievable) {
                return null;
            }

            double actualVelocity = pitchFw.exitVelocityMps;

            ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                    gp,
                    input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                    actualVelocity, pitchRad, requiredYaw,
                    pitchFw.ballSpinRpm,
                    input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                    iterTargetX, iterTargetY, input.getTargetZ(),
                    input.getTargetRadius()
            );

            if (!trajSim.hitTarget && trajSim.maxHeight > input.getTargetZ()) {
                ProjectileMotion.TrajectoryResult refined = refineVelocityForHit(
                        gp,
                        input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                        pitchRad, requiredYaw, pitchFw.ballSpinRpm,
                        input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                        iterTargetX, iterTargetY, input.getTargetZ(),
                        input.getTargetRadius(), actualVelocity);
                if (refined != null) {
                    trajSim = refined;
                    if (refined.trajectory.length > 0) {
                        ProjectileMotion.TrajectoryState s0 = refined.trajectory[0];
                        actualVelocity = Math.sqrt(s0.vx * s0.vx + s0.vy * s0.vy + s0.vz * s0.vz);
                        pitchFw = flywheelSimForPitch.simulateForVelocity(actualVelocity);
                    }
                }
            }

            if (!pitchFw.isAchievable) return null;

            if (requiredClearance > 0 && trajSim.maxHeight < requiredClearance) {
                return null;
            }

            double minArcHeight = input.getMinArcHeightMeters();
            if (minArcHeight > 0 && trajSim.maxHeight < input.getTargetZ() + minArcHeight) {
                return null;
            }

            double hoopTolerance = input.getTargetRadius() * config.getHoopToleranceMultiplier();
            boolean hitsTarget = trajSim.hitTarget
                    || (trajSim.descendingAtClosest && trajSim.closestApproach <= hoopTolerance
                    && trajSim.entryAngleDegrees >= SolverConstants.getMinEntryAngleDegrees());

            boolean isMarginal = !hitsTarget;
            
            if (!hitsTarget && trajSim.closestApproach > 0.5) {
                return null;
            }

            if (isFlyover(trajSim.trajectory, iterTargetX, iterTargetY,
                    input.getTargetZ(), input.getTargetRadius())) {
                return null;
            }

        double missDistanceLocal = (trajSim.horizontalDistAtCrossing >= 0)
            ? trajSim.horizontalDistAtCrossing : trajSim.closestApproach;

    double distanceMeters = Math.hypot(iterTargetX - input.getShooterX(),
        iterTargetY - input.getShooterY());
        double score = computeTrajectoryCandidateScore(input, pitchDeg, trajSim,
            missDistanceLocal, pitchFw.requiredWheelRpm, distanceMeters, requiredYaw);
            
            // Penalize marginal shots so they only win if no perfect hit is possible.
            if (isMarginal) {
                score -= 1000.0;
            }

            return new SweepCandidate(pitchRad, score, iterTargetX, iterTargetY, trajSim.flightTime, isMarginal);
        }).filter(java.util.Objects::nonNull)
                .max(java.util.Comparator.comparingDouble(c -> c.score))
                .orElse(null);

        if (best == null) {
            return null;
        }

        double outPitch = best.pitchRad;
        double outItX = best.itX;
        double outItY = best.itY;
        double outTof = best.tof;
        double marginalFlag = best.isMarginal ? 1.0 : 0.0;

        return new double[]{outPitch, outItX, outItY, outTof, marginalFlag};
    }

    private double[] solveBisectionCore(
            ShotInput input, FlywheelSimulator flywheelSimForPitch, GamePiece gp,
            double effectiveTargetX, double effectiveTargetY, double estimatedTof,
            double distance, double heightDiff, double dragComp,
            double effectiveMinPitch, double effectiveMaxPitch,
            double requiredClearance, boolean moving, SolveDebugInfo debugInfo) {

        double lowPitchDeg = effectiveMinPitch;
        double highPitchDeg = effectiveMaxPitch;

    double bestPitch = Double.NaN;
    double bestScore = -Double.MAX_VALUE;
        double itX = effectiveTargetX, itY = effectiveTargetY;
    double bestTof = estimatedTof;


        for (int i = 0; i < SolverConstants.getMaxIterations(); i++) {
            double midPitchDeg = (lowPitchDeg + highPitchDeg) / 2.0;
            double pitchRad = Math.toRadians(midPitchDeg);

            double iterTargetX = effectiveTargetX;
            double iterTargetY = effectiveTargetY;

            double dx = iterTargetX - input.getShooterX();
            double dy = iterTargetY - input.getShooterY();
            double iterDistance = Math.sqrt(dx * dx + dy * dy);
            double requiredYaw = Math.atan2(dy, dx);

            double vacuumV = calculateRequiredVelocityForPitch(iterDistance, heightDiff, pitchRad);
            if (Double.isNaN(vacuumV) || vacuumV <= 0) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(midPitchDeg, SolveDebugInfo.RejectionReason.ARC_TOO_LOW,
                            Double.MAX_VALUE, 0, 0, false, new ProjectileMotion.TrajectoryState[0]);
                }
                lowPitchDeg = midPitchDeg;
                continue;
            }

            double vacuumHoriz = vacuumV * Math.cos(pitchRad);
            double compensatedHoriz = vacuumHoriz * dragComp;
            double vVert = compensatedHoriz * Math.tan(pitchRad);
            double targetV = Math.sqrt(compensatedHoriz * compensatedHoriz + vVert * vVert);

            FlywheelSimulator.SimulationResult pitchFw = flywheelSimForPitch.simulateForVelocity(targetV);
            double actualVelocity = pitchFw.exitVelocityMps;

            ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulateFast(
                    gp,
                    input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                    actualVelocity, pitchRad, requiredYaw,
                    pitchFw.ballSpinRpm,
                    input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                    iterTargetX, iterTargetY, input.getTargetZ(),
                    input.getTargetRadius()
            );

            // Refine if needed to get correct closest approach
            if (!trajSim.hitTarget && trajSim.maxHeight > input.getTargetZ()) {
                ProjectileMotion.TrajectoryResult refined = refineVelocityForHit(
                        gp,
                        input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                        pitchRad, requiredYaw, pitchFw.ballSpinRpm,
                        input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                        iterTargetX, iterTargetY, input.getTargetZ(),
                        input.getTargetRadius(), actualVelocity);
                if (refined != null && refined.hitTarget) {
                    trajSim = refined;
                    actualVelocity = Math.sqrt(
                            trajSim.trajectory[0].vx * trajSim.trajectory[0].vx
                            + trajSim.trajectory[0].vy * trajSim.trajectory[0].vy
                            + trajSim.trajectory[0].vz * trajSim.trajectory[0].vz);
                    pitchFw = flywheelSimForPitch.simulateForVelocity(actualVelocity);
                }
            }

            double zAtTarget = Double.NaN;
            double targetHorizSq = iterDistance * iterDistance;

            for (ProjectileMotion.TrajectoryState st : trajSim.trajectory) {
                double hd2 = Math.pow(st.x - input.getShooterX(), 2) + Math.pow(st.y - input.getShooterY(), 2);
                if (hd2 >= targetHorizSq) {
                    zAtTarget = st.z;
                    break;
                }
            }

            double missDistance = trajSim.closestApproach;

            if (trajectoryCollides(trajSim, input, input.getShooterX(), input.getShooterY())) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(midPitchDeg, SolveDebugInfo.RejectionReason.COLLISION,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                lowPitchDeg = midPitchDeg;
            } else if (trajSim.maxHeight > input.getTargetZ()) {
                // Candidate may be acceptable; apply stable scoring to reduce rapid switching.
            double score = computeTrajectoryCandidateScore(input,
                midPitchDeg, trajSim, missDistance,
                pitchFw.requiredWheelRpm, iterDistance, requiredYaw);
                if (score > bestScore) {
                    bestScore = score;
                    bestPitch = pitchRad;
                    itX = iterTargetX;
                    itY = iterTargetY;
                    bestTof = trajSim.flightTime > 0 ? trajSim.flightTime : bestTof;
                    if (debugInfo != null) {
                        debugInfo.recordAccepted(midPitchDeg, missDistance,
                                trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime,
                                trajSim.hitTarget, trajSim.trajectory);
                    }
                }
            } else {
                if (debugInfo != null) {
                    debugInfo.recordRejected(midPitchDeg, SolveDebugInfo.RejectionReason.MISSED_TARGET,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
            }

            if (!Double.isNaN(zAtTarget)) {
                if (zAtTarget > input.getTargetZ()) {
                    highPitchDeg = midPitchDeg;
                } else {
                    lowPitchDeg = midPitchDeg;
                }
            } else {
                lowPitchDeg = midPitchDeg;
            }
        }

        if (Double.isNaN(bestPitch)) {
            return null;
        }

        double isMarginal = (bestScore < 0) ? 1.0 : 0.0;
        return new double[]{bestPitch, itX, itY, bestTof, isMarginal};
    }

    /**
     * Computes a composite quality score for a SWEEP candidate. Higher scores
     * indicate better overall trajectory quality.
     *
     * <p>
     * Score components:
     * <ul>
     * <li>Accuracy (40%): how close to target center (miss distance / target
     * radius)</li>
     * <li>Stability (30%): deviation from optimal ~45° pitch, steep angle
     * penalty</li>
     * <li>Speed (20%): lower time-of-flight is better</li>
     * <li>Entry angle (10%): steeper entry drops into hub better</li>
     * </ul>
     *
     * @param pitchDeg candidate pitch in degrees
     * @param missDistance miss distance in meters
     * @param targetRadius target acceptance radius in meters
     * @param timeOfFlight simulated time of flight in seconds
     * @param entryAngleDeg entry angle into target in degrees
     * @return quality score in [0, 100]
     */
    private double computeSweepQualityScore(ShotInput input, double pitchDeg, double missDistance,
        double targetRadius, double timeOfFlight,
        double entryAngleDeg, double requiredWheelRpm,
        double distanceMeters, double maxHeight) {
        double robotVelNorm = Math.hypot(input.getEffectiveRobotVx(), input.getEffectiveRobotVy());
        double accuracyScore;
        if (targetRadius > 0) {
            double relMiss = missDistance / targetRadius;
            accuracyScore = Math.max(0, 40.0 * (1.0 - relMiss));
        } else {
            accuracyScore = missDistance < 0.01 ? 40.0 : 0.0;
        }

        double optimalPitch;
        if (hasTuningPitch) {
            Double val = tuningPitchMap.get(distanceMeters);
            if (val != null) {
                optimalPitch = val;
            } else {
                if (distanceMeters <= 3.5) {
                    optimalPitch = 15.0;
                } else {
                    optimalPitch = 45.0 - Math.min(15.0, Math.max(0.0, (distanceMeters - 3.5) * 1.5));
                }
            }
        } else {
            if (distanceMeters <= 3.5) {
                // Favor shallow angles observed in physical testing (10-22 deg)
                optimalPitch = 15.0;
            } else {
                optimalPitch = 45.0 - Math.min(15.0, Math.max(0.0, (distanceMeters - 3.5) * 1.5));
            }
        }
        double deviation = Math.abs(pitchDeg - optimalPitch);
        double stabilityScore = Math.max(0, 30.0 * (1.0 - deviation / 45.0));
        
        // EMPIRICAL OVERRIDE: Only apply massive bonus if stationary.
        // Tuning points (stationary data) should not override physics when moving.
        if (hasTuningPitch && robotVelNorm < 0.1) {
             stabilityScore = Math.max(0, 1000.0 * (1.0 - deviation / 5.0)); // Huge bonus for matching mapped pitch
        } else if (pitchDeg > 70.0) {
            stabilityScore *= 0.5;
        }

        double speedScore = Math.max(0, 20.0 * (1.0 - timeOfFlight / 3.0));

        double rpmScore = 0;
        if (requiredWheelRpm > 0) {
            if (distanceMeters <= SolverConstants.getCloseRangeThresholdMeters()
                    && requiredWheelRpm < CLOSE_RANGE_MIN_RPM) {
                return -1000.0;
            }
            double idealRpm;
            if (hasTuningRpm) {
                Double val = tuningRpmMap.get(distanceMeters);
                if (val != null) {
                    idealRpm = val;
                } else {
                    if (distanceMeters <= 3.5) {
                        idealRpm = 2100.0;
                    } else if (distanceMeters <= 5.0) {
                        idealRpm = 2100.0 + (distanceMeters - 3.5) * 100.0;
                    } else if (distanceMeters <= 8.0) {
                        idealRpm = 2250.0 + (distanceMeters - 5.0) * 120.0;
                    } else {
                        idealRpm = 2610.0 + (distanceMeters - 8.0) * 120.0;
                    }
                    idealRpm = Math.min(idealRpm, 3400.0);
                }
            } else {
                if (distanceMeters <= 1.5) {
                    idealRpm = 1750.0;
                } else if (distanceMeters <= 3.5) {
                    idealRpm = 1750.0 + (distanceMeters - 1.5) * 325.0;
                } else if (distanceMeters <= 6.0) {
                    idealRpm = 2400.0 + (distanceMeters - 3.5) * 100.0; // 2650 at 6m
                } else {
                    idealRpm = 2650.0 + (distanceMeters - 6.0) * 80.0;
                }
                idealRpm = Math.min(idealRpm, 3000.0);
            }

            double rpmOffset = Math.abs(requiredWheelRpm - idealRpm);
            double rpmScoreFromIdeal = Math.max(0, 25.0 * (1.0 - rpmOffset / 1200.0));

            // EMPIRICAL OVERRIDE: Only apply massive bonus if stationary.
            // Tuning points (stationary data) should not override physics when moving.
            if (hasTuningRpm && robotVelNorm < 0.1) {
                 rpmScoreFromIdeal = Math.max(0, 1000.0 * (1.0 - rpmOffset / 100.0)); // Huge bonus for matching mapped rpm
            }
            rpmScore = rpmScoreFromIdeal;

            if (!hasTuningRpm) {
                // Discourage excessive RPM even when feasibility allows it.
                // Penalty kicks in earlier and is steeper to keep shots in the 2-2.5k range.
                if (requiredWheelRpm > 2500.0) {
                    double over = requiredWheelRpm - 2500.0;
                    rpmScore -= Math.min(50.0, over / 30.0); // -50 max penalty for >4000.
                }

                if (distanceMeters <= 6.5 && requiredWheelRpm > 2600.0) {
                    double over = requiredWheelRpm - 2600.0;
                    rpmScore -= Math.min(60.0, over * 0.3);
                }

                // Aggressively discourage mid-range shots from settling in 3k+ RPM zones.
                if (distanceMeters <= 6.5) {
                    if (requiredWheelRpm > 3200.0) {
                        rpmScore -= 150.0;
                    } else if (requiredWheelRpm > 3000.0) {
                        rpmScore -= 100.0;
                    } else if (requiredWheelRpm > 2800.0) {
                        rpmScore -= 50.0;
                    }
                }
            }
        }

        double entryScore = Math.min(10.0, entryAngleDeg / 9.0);

        return accuracyScore + stabilityScore + speedScore + rpmScore + entryScore;
    }

    private double computeTrajectoryCandidateScore(ShotInput input,
        double pitchDeg,
        ProjectileMotion.TrajectoryResult trajSim,
        double missDistance,
        double requiredWheelRpm,
        double distanceMeters,
        double candidateYawRadians) {
        double score = computeSweepQualityScore(input, pitchDeg,
                missDistance, input.getTargetRadius(),
                trajSim.flightTime, trajSim.entryAngleDegrees,
                requiredWheelRpm, distanceMeters, trajSim.maxHeight);

        // Prefer a user-specified arc preference; this keeps close-range behavior
        // from swapping between low/high arcs.
        if (input != null && input.getPreferredArcHeightMeters() > 0) {
            double preferredHeight = input.getPreferredArcHeightMeters();
            double delta = Math.abs(trajSim.maxHeight - preferredHeight);
            double arcScore = Math.max(0.0, 15.0 * (1.0 - delta / Math.max(preferredHeight, 0.1)));
            score += input.getArcBiasStrength() * arcScore;
        }

        // Add a small continuity bias to reduce shot-to-shot jitter when close.
        if (input != null && distanceMeters <= SolverConstants.getCloseRangeThresholdMeters() * 1.5
            && !Double.isNaN(previousPitchAngleRadians)) {
            double prevPitchDeg = Math.toDegrees(previousPitchAngleRadians);
            double pitchDelta = Math.abs(pitchDeg - prevPitchDeg);
            double continuityBonus = Math.max(0.0, 10.0 * (1.0 - Math.min(pitchDelta, 30.0) / 30.0));
            score += continuityBonus;
        }

        if (!Double.isNaN(previousYawAdjustmentRadians)) {
            double yawDiff = Math.abs(candidateYawRadians - previousYawAdjustmentRadians);
            double yawDeltaDeg = Math.toDegrees(Math.atan2(Math.sin(yawDiff), Math.cos(yawDiff)));
            double yawBonus = Math.max(0.0, 5.0 * (1.0 - Math.min(Math.abs(yawDeltaDeg), 20.0) / 20.0));
            score += yawBonus;
        }

        if (!Double.isNaN(previousRpm)) {
            double rpmDelta = Math.abs(requiredWheelRpm - previousRpm);
            double rpmBonus = Math.max(0.0, 8.0 * (1.0 - Math.min(rpmDelta, 500.0) / 500.0));
            score += rpmBonus;
        }

        return score;
    }

    /**
     * Creates a new trajectory solver builder.
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Internal candidate for sweep searches.
     */
    private static class SweepCandidate {

        final double pitchRad;
        final double score;
        final double itX, itY, tof;
        final boolean isMarginal;

        SweepCandidate(double pitchRad, double score, double itX, double itY, double tof, boolean isMarginal) {
            this.pitchRad = pitchRad;
            this.score = score;
            this.itX = itX;
            this.itY = itY;
            this.tof = tof;
            this.isMarginal = isMarginal;
        }
    }

    /**
     * Fluent builder for constructing a {@link TrajectorySolver}.
     */
    public static class Builder {

        private GamePiece gamePiece = GamePieces.getCurrent();
        private SolverConfig config = SolverConfig.defaults();

        /**
         * Sets the game piece for trajectory calculations.
         */
        public Builder gamePiece(GamePiece val) {
            this.gamePiece = val;
            return this;
        }

        /**
         * Sets the solver configuration.
         */
        public Builder config(SolverConfig val) {
            this.config = val;
            return this;
        }

        /**
         * Builds the {@link TrajectorySolver} instance.
         */
        public TrajectorySolver build() {
            return new TrajectorySolver(gamePiece, config);
        }
    }

    /**
     * Creates a solver for the given game piece.
     */
    public TrajectorySolver(GamePiece gamePiece) {
        this(gamePiece, SolverConfig.defaults());
    }

    /**
     * Creates a solver with custom configuration.
     */
    public TrajectorySolver(GamePiece gamePiece, SolverConfig config) {
        this.gamePiece = gamePiece;
        this.config = config;

        AirResistance airResistance = new AirResistance(true, 1.18,
                ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.FOAM_BALL_DRAG_COEFFICIENT, true);
        this.projectileMotion = new ProjectileMotion(airResistance,
                config.getSimulationTimeStep(), config.getFastSimulationTimeStep());
        this.flywheelGenerator = new FlywheelGenerator(gamePiece, config.getFlywheelGenParams());
    }

    /**
     * Creates a solver optimized for 2026 REBUILT game.
     *
     * @return A new TrajectorySolver configured for 2026 REBUILT
     */
    public static TrajectorySolver forGame2026() {
        return forGamePiece(GamePieces.REBUILT_2026_BALL);
    }

    /**
     * Creates a solver for a specific game year.
     *
     * @param year The FRC game year
     * @return A new TrajectorySolver configured for that year's game piece
     */
    public static TrajectorySolver forYear(int year) {
        GamePiece piece = GamePieces.getByYear(year);
        if (piece == null) {
            piece = GamePieces.getCurrent();
        }
        return forGamePiece(piece);
    }

    /**
     * Creates a solver for a specific game piece.
     *
     * @param gamePiece The game piece to use
     * @return A new TrajectorySolver configured for that game piece
     */
    public static TrajectorySolver forGamePiece(GamePiece gamePiece) {
        return new TrajectorySolver(gamePiece);
    }

    /**
     * Solves for the trajectory. Main entry point.
     *
     * @param input shot parameters
     * @return result with recommended pitch, RPM, etc.
     */
    public TrajectoryResult solve(ShotInput input) {
        if (input == null) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.INVALID_INPUT,
                    "Shot input cannot be null",
                    input
            );
        }

        double fixedDistance = input.getHorizontalDistanceMeters();

        // Removed the direct tuning-map bypass. 
        // We now allow the solver core to run even if tuning points exist, 
        // using them as a high-weight scoring bias in computeSweepQualityScore 
        // instead of a hard override. This ensures physics checks are always performed.

        boolean moving = Math.abs(input.getEffectiveRobotVx()) > SolverConstants.getMovementThresholdMps()
                || Math.abs(input.getEffectiveRobotVy()) > SolverConstants.getMovementThresholdMps();

        // Cache the flywheel in a local variable so parallel precompute runs don't
        // race on the shared solver state.
        FlywheelConfig cachedFlywheelSnapshot = cachedFlywheel;
        int convergenceIterations = moving
                ? SolverConstants.getMovingConvergenceIterations()
                : SolverConstants.getStationaryIterations();

        double distance = fixedDistance;
        double estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();        double effectiveTargetX = input.getTargetX();
        double effectiveTargetY = input.getTargetY();

        for (int i = 0; i < convergenceIterations; i++) {
            if (moving) {
                effectiveTargetX = input.getTargetX() - input.getEffectiveRobotVx() * estimatedTof;
                effectiveTargetY = input.getTargetY() - input.getEffectiveRobotVy() * estimatedTof;
                double dx = effectiveTargetX - input.getShooterX();
                double dy = effectiveTargetY - input.getShooterY();
                distance = Math.sqrt(dx * dx + dy * dy);
                estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
            }
        }

        double heightDiff = input.getHeightDifferenceMeters();

        if (distance < SolverConstants.getMinTargetDistanceMeters()) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.INVALID_INPUT,
                    String.format("Target too close (< %.2fm)", SolverConstants.getMinTargetDistanceMeters()),
                    input
            );
        }

        boolean pathCrossesObstacle = input.pathRequiresArc();
        double requiredClearance = input.getRequiredClearanceHeight();

        double effectiveMinPitch = input.getMinPitchDegrees();
        double effectiveMaxPitch = input.getMaxPitchDegrees();

        if (pathCrossesObstacle) {
            double forcedMin = SolverConstants.getForceHighArcMinPitchDegrees();
            if (effectiveMaxPitch - forcedMin >= MIN_FORCED_ARC_RANGE_DEG) {
                effectiveMinPitch = Math.max(effectiveMinPitch, forcedMin);
            }
        }

        double dragComp = calculateDragCompensation(distance);

        double minVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff);
        boolean isCloseRange = distance < SolverConstants.getCloseRangeThresholdMeters();
        double velocityBuffer = isCloseRange
                ? SolverConstants.getCloseRangeVelocityMultiplier()
                : SolverConstants.getVelocityBufferMultiplier() * dragComp;
        double representativeVelocity = minVelocity * velocityBuffer;
        double maxPitchV = calculateRequiredVelocityForPitch(distance, heightDiff,
                Math.toRadians(effectiveMaxPitch));
        if (!Double.isNaN(maxPitchV) && maxPitchV > 0) {
            representativeVelocity = Math.max(representativeVelocity, maxPitchV);
        }
        if (cachedFlywheelSnapshot != null) {
            double absoluteMinWithBuffer = minVelocity * (isCloseRange ? 1.1 : 1.15 * dragComp);
            representativeVelocity = Math.min(representativeVelocity, absoluteMinWithBuffer);
        }

        FlywheelGenerator.GenerationResult genResult;
        if (cachedFlywheelSnapshot != null) {
            FlywheelSimulator simulator = new FlywheelSimulator(cachedFlywheelSnapshot, gamePiece);
            FlywheelSimulator.SimulationResult simResult = simulator.simulateForVelocity(representativeVelocity);

            if (simResult.isAchievable) {
                genResult = new FlywheelGenerator.GenerationResult(
                        List.of(new FlywheelGenerator.ScoredConfig(
                                cachedFlywheelSnapshot, simResult,
                                simulator.scoreConfiguration(representativeVelocity)
                        )), 1
                );
            } else {
                cachedFlywheelSnapshot = null;
                cachedFlywheel = null;
                genResult = flywheelGenerator.generateAndEvaluate(representativeVelocity);
            }
        } else {
            genResult = flywheelGenerator.generateAndEvaluate(representativeVelocity);
        }

        if (genResult.achievableCount == 0) {
            genResult = flywheelGenerator.evaluatePresets(representativeVelocity);
        }

        double baseVelocity = minVelocity * velocityBuffer;
        if (genResult.achievableCount == 0 && representativeVelocity > baseVelocity) {
            genResult = flywheelGenerator.generateAndEvaluate(baseVelocity);
            if (genResult.achievableCount == 0) {
                genResult = flywheelGenerator.evaluatePresets(baseVelocity);
            }
        }

        if (genResult.achievableCount == 0) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.VELOCITY_EXCEEDED,
                    String.format("No flywheel can achieve required velocity: %.2f m/s", baseVelocity),
                    input
            );
        }

    FlywheelGenerator.ScoredConfig bestFlywheel = genResult.bestConfig;
    FlywheelConfig flywheel = bestFlywheel.config;

    // Update the shared cache once the best config is known. Using a local
    // snapshot above avoids races during the solve sweep.
    cachedFlywheel = flywheel;

        FlywheelSimulator flywheelSimForPitch = new FlywheelSimulator(flywheel, gamePiece);

        double bestPitchAngle = Double.NaN;
        ProjectileMotion.TrajectoryResult bestTrajSim = null;
        FlywheelSimulator.SimulationResult bestFlywheelSim = null;

        SolveDebugInfo debugInfo = debugEnabled ? new SolveDebugInfo() : null;

        double[] coreResult;
        TrajectoryResult.Status outStatus = TrajectoryResult.Status.SUCCESS;
        String outMessage = "Valid trajectory found";

        if (solveMode == SolveMode.SWEEP) {
            coreResult = solveSweepCore(input, flywheelSimForPitch, gamePiece,
                    effectiveTargetX, effectiveTargetY, estimatedTof,
                    distance, heightDiff, dragComp,
                    effectiveMinPitch, effectiveMaxPitch,
                    requiredClearance, moving, debugInfo);
        } else if (solveMode == SolveMode.BISECTION) {
            coreResult = solveBisectionCore(input, flywheelSimForPitch, gamePiece,
                    effectiveTargetX, effectiveTargetY, estimatedTof,
                    distance, heightDiff, dragComp,
                    effectiveMinPitch, effectiveMaxPitch,
                    requiredClearance, moving, debugInfo);
        } else if (solveMode == SolveMode.HYBRID) {
            ca.team4308.absolutelib.math.trajectories.shooter.ShotParameters precomputed = null;
            if (input.getMap() != null) {
                precomputed = input.getMap().lookup(distance);
            }

            if (precomputed != null && precomputed.valid) {
                double seedPitch = precomputed.pitchDegrees;
                double span = 2.0; // Search +/- 1 degree around the seed
                coreResult = solveBisectionCore(
                        input, flywheelSimForPitch, gamePiece,
                        effectiveTargetX, effectiveTargetY, estimatedTof,
                        distance, heightDiff, dragComp,
                        seedPitch - span / 2.0, seedPitch + span / 2.0,
                        requiredClearance, moving, debugInfo);
                if (coreResult == null) {
                    // Fallback to precomputed if bisection fails
                    coreResult = new double[]{
                        Math.toRadians(seedPitch), effectiveTargetX, effectiveTargetY, estimatedTof
                    };
                }
            } else {
                // Fallback to bisection if no map
                coreResult = solveBisectionCore(input, flywheelSimForPitch, gamePiece,
                        effectiveTargetX, effectiveTargetY, estimatedTof,
                        distance, heightDiff, dragComp,
                        effectiveMinPitch, effectiveMaxPitch,
                        requiredClearance, moving, debugInfo);
            }
        } else {

            coreResult = solveConstraintCore(input, flywheelSimForPitch, gamePiece,
                    effectiveTargetX, effectiveTargetY, estimatedTof,
                    distance, heightDiff, dragComp,
                    effectiveMinPitch, effectiveMaxPitch,
                    requiredClearance, moving, debugInfo);
            if (coreResult == null) {

                coreResult = solveBisectionCore(input, flywheelSimForPitch, gamePiece,
                        effectiveTargetX, effectiveTargetY, estimatedTof,
                        distance, heightDiff, dragComp,
                        effectiveMinPitch, effectiveMaxPitch,
                        requiredClearance, moving, debugInfo);
            }
        }

        if (coreResult != null) {
            bestPitchAngle = coreResult[0];
            effectiveTargetX = coreResult[1];
            effectiveTargetY = coreResult[2];
            estimatedTof = coreResult[3];
            boolean isMarginalSolution = coreResult.length > 4 && coreResult[4] > 0.5;
            
            outStatus = isMarginalSolution ? TrajectoryResult.Status.MARGINAL : TrajectoryResult.Status.SUCCESS;
            outMessage = isMarginalSolution ? "Marginal trajectory found (best effort)" : "Valid trajectory found";

            double dx2 = effectiveTargetX - input.getShooterX();
            double dy2 = effectiveTargetY - input.getShooterY();
            double bestYaw = Math.atan2(dy2, dx2);
            double bestIterDist = Math.sqrt(dx2 * dx2 + dy2 * dy2);

            double bestVacV = calculateRequiredVelocityForPitch(bestIterDist, heightDiff, bestPitchAngle);
            if (Double.isNaN(bestVacV) || bestVacV <= 0) {

            } else {
                double bestDragComp = 1.0 + (dragComp - 1.0) * Math.cos(bestPitchAngle);
                FlywheelSimulator.SimulationResult fw = flywheelSimForPitch.simulateForVelocity(bestVacV * bestDragComp);
                if (fw.isAchievable) {
                    ProjectileMotion.TrajectoryResult traj = projectileMotion.simulate(
                            gamePiece,
                            input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                            fw.exitVelocityMps, bestPitchAngle, bestYaw, fw.ballSpinRpm,
                            input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                            effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                            input.getTargetRadius());

                    if (!traj.hitTarget && traj.maxHeight > input.getTargetZ()) {
                        ProjectileMotion.TrajectoryResult refined = refineVelocityForHit(
                                gamePiece,
                                input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                                bestPitchAngle, bestYaw, fw.ballSpinRpm,
                                input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                                effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                                input.getTargetRadius(), fw.exitVelocityMps);
                        if (refined != null) {
                            traj = refined;
                            double refV = fw.exitVelocityMps;
                            if (refined.trajectory.length > 0) {
                                ProjectileMotion.TrajectoryState s0 = refined.trajectory[0];
                                refV = Math.sqrt(s0.vx * s0.vx + s0.vy * s0.vy + s0.vz * s0.vz);
                            }
                            fw = flywheelSimForPitch.simulateForVelocity(refV);
                        }
                    }

                    if (fw.isAchievable) {
                        boolean finalValid = true;

                        if (trajectoryCollides(traj, input, input.getShooterX(), input.getShooterY())) {
                            finalValid = false;
                        }
                        if (finalValid && isFlyover(traj.trajectory, effectiveTargetX, effectiveTargetY,
                                input.getTargetZ(), input.getTargetRadius())) {
                            finalValid = false;
                        }

                        if (finalValid) {
                            bestTrajSim = traj;
                            bestFlywheelSim = fw;
                        }
                    }
                }
            }
        }

        if (Double.isNaN(bestPitchAngle) || bestTrajSim == null) {
            String reason = pathCrossesObstacle
                    ? "No collision-free trajectory found; all angles hit obstacles or miss target"
                    : "No trajectory solution exists for given distance and velocity";
            TrajectoryResult failResult = TrajectoryResult.failure(
                    TrajectoryResult.Status.OUT_OF_RANGE,
                    reason,
                    input
            );
            if (debugInfo != null) {
                failResult.setDebugInfo(debugInfo);
            }
            return failResult;
        }

        double pitchDegrees = Math.toDegrees(bestPitchAngle);

        double dx = effectiveTargetX - input.getShooterX();
        double dy = effectiveTargetY - input.getShooterY();
        double requiredYaw = Math.atan2(dy, dx);
        double yawAdjustment = requiredYaw - input.getShooterYaw();
        while (yawAdjustment > Math.PI) {
            yawAdjustment -= 2 * Math.PI;
        }
        while (yawAdjustment < -Math.PI) {
            yawAdjustment += 2 * Math.PI;
        }

        double requiredRpm = bestFlywheelSim.requiredWheelRpm;
    
        if (empiricalMap != null && empiricalMap.hasData()) {
            EmpiricalShotMap.QueryResult mapRpmResult = empiricalMap.query(distance);
            if (mapRpmResult.inRange) {
                requiredRpm = mapRpmResult.rpm;
            } else {
                double edgeDist = distance < empiricalMap.getMinDistance()
                        ? empiricalMap.getMinDistance()
                        : empiricalMap.getMaxDistance();
                EmpiricalShotMap.QueryResult edgeResult = empiricalMap.query(edgeDist);

                double edgeVacV = calculateRequiredVelocityForPitch(
                        edgeDist, heightDiff, bestPitchAngle);
                if (!Double.isNaN(edgeVacV) && edgeVacV > 0) {
                    double edgeDragComp = 1.0 + (dragComp - 1.0) * Math.cos(bestPitchAngle);
                    FlywheelSimulator.SimulationResult edgeFw =
                            flywheelSimForPitch.simulateForVelocity(edgeVacV * edgeDragComp);
                    if (edgeFw.isAchievable && edgeFw.requiredWheelRpm > 0) {
                        double correctionFactor = edgeResult.rpm / edgeFw.requiredWheelRpm;
                        double extrapolationDist = Math.abs(distance - edgeDist);
                        double blendRange = 3.0;
                        double blend = Math.max(0.0, 1.0 - extrapolationDist / blendRange);
                        double blendedFactor = 1.0 + blend * (correctionFactor - 1.0);
                        requiredRpm = requiredRpm * blendedFactor;
                    }
                }
            }
        }

        // Apply adaptive calibration adjustment from real-time trajectory feedback
        // This allows the system to learn the actual RPM-to-velocity relationship
        // instead of relying on assumptions about wheel size or gear ratio.
        if (calibrationSamples.size() >= MIN_CALIBRATION_SAMPLES_FOR_CONFIDENCE) {
            double adaptiveFactor = getAdaptiveRpmFactor();
            double defaultFactor = 0.01532; // From DefaultShotTable
            if (adaptiveFactor != defaultFactor) {
                double calibrationRatio = adaptiveFactor / defaultFactor;
                requiredRpm = requiredRpm * calibrationRatio;
            }
        }

        requiredRpm = Math.max(config.getMinRpm(), Math.min(config.getMaxRpm(), requiredRpm));
        double actualVelocity = bestFlywheelSim.exitVelocityMps;
        double timeOfFlight = bestTrajSim.flightTime;
        double maxHeight = bestTrajSim.maxHeight;
        double marginOfError = bestTrajSim.closestApproach;

        // Stop tweakin pls
    double enforcedMinRpm = Math.max(CLOSE_RANGE_MIN_RPM, config.getMinRpm());
    if (Double.isNaN(pitchDegrees) || pitchDegrees < Math.max(1.0, input.getMinPitchDegrees())
        || Double.isNaN(requiredRpm) || requiredRpm <= Math.max(100.0, enforcedMinRpm)) {
        return TrajectoryResult.failure(
            TrajectoryResult.Status.OUT_OF_RANGE,
            "Trajectory solution is not valid under system constraints (likely oscillatory near target)",
            input
        );
    }

        TrajectoryResult.DiscreteShot discreteSolution = new TrajectoryResult.DiscreteShot(
                requiredRpm, pitchDegrees,
                (int) (requiredRpm / config.getCrtRpmResolution()),
                (int) (pitchDegrees / config.getCrtAngleResolution()),
                50.0
        );

    double confidence = calculateConfidence(
        bestFlywheel.score,
        bestTrajSim.hitTarget,
        marginOfError,
        input.getTargetRadius(),
        discreteSolution.score,
        requiredRpm,
        distance
    );

        TrajectoryResult successResult = new TrajectoryResult(
                outStatus, outMessage,
                input, gamePiece,
                bestPitchAngle, yawAdjustment, actualVelocity,
                flywheel, bestFlywheelSim, requiredRpm,
                timeOfFlight, maxHeight, marginOfError,
                discreteSolution, confidence
        );
    previousPitchAngleRadians = bestPitchAngle;
    previousYawAdjustmentRadians = yawAdjustment;
    previousRpm = requiredRpm;
        if (debugInfo != null) {
            successResult.setDebugInfo(debugInfo);
        }
        return successResult;
    }

    /**
     * Solves for multiple velocities to show trajectory options. Generates
     * solutions across a range of possible velocities.
     *
     * @param input Shot input
     * @param velocitySteps Number of velocity steps to try
     * @return Array of trajectory results at different velocities
     */
    public TrajectoryResult[] solveRange(ShotInput input, int velocitySteps) {
        double distance = input.getHorizontalDistanceMeters();
        double heightDiff = input.getHeightDifferenceMeters();

        double minVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff)
                * calculateDragCompensation(distance);
        double maxVelocity = minVelocity * SolverConstants.getMaxVelocityRangeMultiplier();

        TrajectoryResult[] results = new TrajectoryResult[velocitySteps];

        FlywheelGenerator.GenerationResult genResult
                = flywheelGenerator.generateForVelocityRange(
                        minVelocity * (1.0 / SolverConstants.getMinVelocityRangeMultiplier()),
                        maxVelocity * SolverConstants.getMinVelocityRangeMultiplier());

        for (int i = 0; i < velocitySteps; i++) {
            double targetVelocity = minVelocity + (maxVelocity - minVelocity) * i / (velocitySteps - 1);

            FlywheelConfig bestFlywheel = null;
            double bestScore = -1;

            for (FlywheelGenerator.ScoredConfig scored : genResult.configurations) {
                FlywheelSimulator sim = new FlywheelSimulator(scored.config, gamePiece);
                FlywheelSimulator.SimulationResult simResult = sim.simulateForVelocity(targetVelocity);
                if (simResult.isAchievable && scored.score > bestScore) {
                    bestFlywheel = scored.config;
                    bestScore = scored.score;
                }
            }

            if (bestFlywheel != null) {
                results[i] = solveWithFlywheel(input, bestFlywheel);
            } else {
                results[i] = solve(input);
            }
        }

        return results;
    }

    /**
     * Solves using a specific flywheel configuration.
     */
    public TrajectoryResult solveWithFlywheel(ShotInput input, FlywheelConfig flywheel) {
        FlywheelConfig previousCache = cachedFlywheel;
        cachedFlywheel = flywheel;

        try {
            return solve(input);
        } finally {
            cachedFlywheel = previousCache;
        }
    }

    /**
     * Evaluates an existing configuration against a shot.
     */
    public TrajectoryResult evaluate(ShotInput input, FlywheelConfig flywheel, double rpm, double pitchDegrees) {
        FlywheelSimulator simulator = new FlywheelSimulator(flywheel, gamePiece);
        FlywheelSimulator.SimulationResult simResult = simulator.simulateAtRpm(rpm);

        if (!simResult.isAchievable) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.VELOCITY_EXCEEDED,
                    "Specified RPM not achievable with this flywheel",
                    input
            );
        }

        double pitchRadians = Math.toRadians(pitchDegrees);
        double velocity = simResult.exitVelocityMps;

        ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                gamePiece,
                input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                velocity, pitchRadians, input.getRequiredYawRadians(),
                simResult.ballSpinRpm,
                input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                input.getTargetX(), input.getTargetY(), input.getTargetZ(),
                input.getTargetRadius()
        );

        double confidence = trajSim.hitTarget ? 90.0
                : Math.max(0, 70.0 - trajSim.closestApproach * 100.0);

        TrajectoryResult.DiscreteShot discreteSolution
                = new TrajectoryResult.DiscreteShot(
                        rpm, pitchDegrees,
                        (int) (rpm / config.getCrtRpmResolution()),
                        (int) (pitchDegrees / config.getCrtAngleResolution()),
                        confidence / 2
                );

        return new TrajectoryResult(
                input, gamePiece,
                pitchRadians, input.getYawAdjustmentRadians(), velocity,
                flywheel, simResult, rpm,
                trajSim.flightTime, trajSim.maxHeight, trajSim.closestApproach,
                discreteSolution, confidence
        );
    }

    /**
     * Finds the best pitch at a given flywheel RPM (e.g. for rapid-fire when
     * the flywheel is slower than ideal).
     *
     * @param input shot parameters
     * @param flywheel flywheel config to simulate
     * @param currentRpm measured RPM from encoder
     * @return result at the given RPM, or failure if unreachable
     */
    public TrajectoryResult solveAtCurrentRpm(ShotInput input, FlywheelConfig flywheel, double currentRpm) {
        if (input == null) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.INVALID_INPUT,
                    "Shot input cannot be null",
                    input
            );
        }

        FlywheelSimulator simulator = new FlywheelSimulator(flywheel, gamePiece);
        FlywheelSimulator.SimulationResult simResult = simulator.simulateAtRpm(currentRpm);

        if (!simResult.isAchievable) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.VELOCITY_EXCEEDED,
                    String.format("Current RPM (%.0f) not achievable with this flywheel", currentRpm),
                    input
            );
        }

        double actualVelocity = simResult.exitVelocityMps;
        double ballSpin = simResult.ballSpinRpm;

        boolean moving = Math.abs(input.getEffectiveRobotVx()) > SolverConstants.getMovementThresholdMps()
                || Math.abs(input.getEffectiveRobotVy()) > SolverConstants.getMovementThresholdMps();
        int convergenceIterations = moving
                ? SolverConstants.getMovingConvergenceIterations()
                : SolverConstants.getStationaryIterations();

        double distance = input.getHorizontalDistanceMeters();
        double estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();

        double effectiveTargetX = input.getTargetX();
        double effectiveTargetY = input.getTargetY();

        // Removed iterative moving blocks due to vector injection


        if (distance < SolverConstants.getMinTargetDistanceMeters()) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.INVALID_INPUT,
                    String.format("Target too close (< %.2fm)", SolverConstants.getMinTargetDistanceMeters()),
                    input
            );
        }

        boolean pathCrossesObstacle = input.pathRequiresArc();
        double requiredClearance = input.getRequiredClearanceHeight();

        double effectiveMinPitch = input.getMinPitchDegrees();
        double effectiveMaxPitch = input.getMaxPitchDegrees();

        if (pathCrossesObstacle) {
            double forcedMin = SolverConstants.getForceHighArcMinPitchDegrees();
            if (effectiveMaxPitch - forcedMin >= MIN_FORCED_ARC_RANGE_DEG) {
                effectiveMinPitch = Math.max(effectiveMinPitch, forcedMin);
            }
        }

        double bestPitchAngle = Double.NaN;
        ProjectileMotion.TrajectoryResult bestTrajSim = null;

        double g = ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.GRAVITY;
        double v = actualVelocity;
        double d = distance;
        double h = input.getHeightDifferenceMeters();
        double disc = v * v * v * v - g * (g * d * d + 2.0 * h * v * v);

        if (disc >= 0) {
            double pitchRad = Math.atan((v * v + Math.sqrt(disc)) / (g * d));

            pitchRad = Math.max(Math.toRadians(effectiveMinPitch),
                    Math.min(Math.toRadians(effectiveMaxPitch), pitchRad));

            double dx = effectiveTargetX - input.getShooterX();
            double dy = effectiveTargetY - input.getShooterY();
            double requiredYaw = Math.atan2(dy, dx);

            ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                    gamePiece,
                    input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                    actualVelocity, pitchRad, requiredYaw, ballSpin,
                    input.getEffectiveRobotVx(), input.getEffectiveRobotVy(),
                    effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                    input.getTargetRadius()
            );

            if (trajSim.flightTime > 0) {
                estimatedTof = trajSim.flightTime;
            }

            boolean valid = true;
            if (trajectoryCollides(trajSim, input, input.getShooterX(), input.getShooterY())) {
                valid = false;
            }
            if (valid && requiredClearance > 0 && trajSim.maxHeight < requiredClearance) {
                valid = false;
            }
            double minArcHeight = input.getMinArcHeightMeters();
            if (valid && minArcHeight > 0 && trajSim.maxHeight < input.getTargetZ() + minArcHeight) {
                valid = false;
            }

            double hoopTolerance = input.getTargetRadius() * config.getHoopToleranceMultiplier();
            boolean hitsTarget = trajSim.hitTarget
                    || (trajSim.descendingAtClosest && trajSim.closestApproach <= hoopTolerance
                    && trajSim.entryAngleDegrees >= SolverConstants.getMinEntryAngleDegrees());
            if (valid && !hitsTarget) {
                valid = false;
            }
            if (valid && isFlyover(trajSim.trajectory, effectiveTargetX, effectiveTargetY,
                    input.getTargetZ(), input.getTargetRadius())) {
                valid = false;
            }

            if (valid) {
                bestPitchAngle = pitchRad;
                bestTrajSim = trajSim;
            }
        }

        if (Double.isNaN(bestPitchAngle) || bestTrajSim == null) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.OUT_OF_RANGE,
                    String.format("No trajectory found at current RPM (%.0f, exit velocity %.2f m/s)",
                            currentRpm, actualVelocity),
                    input
            );
        }

        double pitchDegrees = Math.toDegrees(bestPitchAngle);

        double dx = effectiveTargetX - input.getShooterX();
        double dy = effectiveTargetY - input.getShooterY();
        double requiredYaw = Math.atan2(dy, dx);
        double yawAdjustment = requiredYaw - input.getShooterYaw();
        while (yawAdjustment > Math.PI) {
            yawAdjustment -= 2 * Math.PI;
        }
        while (yawAdjustment < -Math.PI) {
            yawAdjustment += 2 * Math.PI;
        }

        TrajectoryResult.DiscreteShot discreteSolution = new TrajectoryResult.DiscreteShot(
                currentRpm, pitchDegrees,
                (int) (currentRpm / config.getCrtRpmResolution()),
                (int) (pitchDegrees / config.getCrtAngleResolution()),
                50.0
        );

    double confidence = calculateConfidence(
        simulator.scoreConfiguration(actualVelocity),
        bestTrajSim.hitTarget,
        bestTrajSim.closestApproach,
        input.getTargetRadius(),
        discreteSolution.score,
        currentRpm,
        distance
    );

        return new TrajectoryResult(
                input, gamePiece,
                bestPitchAngle, yawAdjustment, actualVelocity,
                flywheel, simResult, currentRpm,
                bestTrajSim.flightTime, bestTrajSim.maxHeight, bestTrajSim.closestApproach,
                discreteSolution, confidence
        );
    }

    /**
     * Gets the optimal flywheel configuration for a velocity range.
     */
    public FlywheelConfig getOptimalFlywheel(double minVelocityMps, double maxVelocityMps) {
        FlywheelGenerator.GenerationResult result
                = flywheelGenerator.generateForVelocityRange(minVelocityMps, maxVelocityMps);

        return result.bestConfig != null ? result.bestConfig.config : null;
    }

    /**
     * Calculates confidence score for a solution.
     */
    private double calculateConfidence(double flywheelScore, boolean hitTarget,
        double marginOfError, double targetRadius,
        double crtScore, double requiredWheelRpm,
        double distanceMeters) {
        double confidence = 0;

        confidence += Math.min(30, flywheelScore / 5);

        if (hitTarget) {

            confidence += 40;
        } else {
            double relativeError = marginOfError / targetRadius;
            confidence += Math.max(0, 30 - relativeError * 20);
        }

        confidence += Math.min(20, crtScore / 5);

        // Penalize high RPM primarily for short shots; allow higher RPM for long-distance passes.
        if (requiredWheelRpm > 0) {
            double rpmPenalty = Math.max(0.0, requiredWheelRpm - 3000.0) / 1000.0;
            // Fade penalty linearly from 1.0 at ~2m to 0.0 at ~6m.
            double distanceFactor = Math.max(0.0, Math.min(1.0, 1.0 - (distanceMeters - 2.0) / 4.0));
            confidence -= Math.min(20.0, rpmPenalty * 10.0) * distanceFactor;
        }

        confidence += 10;

        return Math.min(100, confidence);
    }

    /**
     * Clears the cached flywheel configuration.
     */
    public void clearCache() {
        cachedFlywheel = null;
    }

    /**
     * Sets a specific flywheel to use for all future solves.
     */
    public void setFlywheel(FlywheelConfig flywheel) {
        cachedFlywheel = flywheel;
    }

    /**
     * Returns the game piece this solver is configured for.
     */
    public GamePiece getGamePiece() {
        return gamePiece;
    }

    /**
     * Returns the solver configuration.
     */
    public SolverConfig getConfig() {
        return config;
    }

    /**
     * Returns the underlying projectile motion simulator.
     */
    public ProjectileMotion getProjectileMotion() {
        return projectileMotion;
    }

    /**
     * Returns the flywheel generator used by this solver.
     */
    public FlywheelGenerator getFlywheelGenerator() {
        return flywheelGenerator;
    }

    /**
     * Returns the currently cached flywheel config, or null if none.
     */
    public FlywheelConfig getCachedFlywheel() {
        return cachedFlywheel;
    }
}
// To many hours were spent making this - Nicholas 2026, Never again
