package ca.team4308.absolutelib.math.trajectories;

import java.util.List;

import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelGenerator;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelSimulator;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;
import ca.team4308.absolutelib.math.trajectories.physics.AirResistance;
import ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion;

/**
 * Main trajectory solver API for FRC turret shooting.
 * 
 * Integrates all trajectory subsystems:
 * - Projectile motion physics
 * - Flywheel simulation and generation
 * - Chinese Remainder Theorem for discrete solutions
 * 
 * <h2>New Multi-Candidate API (Recommended)</h2>
 * <pre>
 * TrajectorySolver solver = TrajectorySolver.forGame2026();
 * 
 * ShotInput input = ShotInput.builder()
 *     .shooterPositionMeters(1.0, 2.0, 0.5)
 *     .targetPositionMeters(5.0, 5.0, 2.5)
 *     .shotPreference(ShotInput.ShotPreference.FASTEST)
 *     .build();
 * 
 * // Get all possible shots sorted by confidence
 * ShotCandidateList candidates = solver.findAllCandidates(input);
 * 
 * // Or just get the best pitch angle
 * double bestPitch = solver.solveBestPitchDegrees(input);
 * </pre>
 * 
 * <h2>Legacy API (Still Supported)</h2>
 * <pre>
 * ShotInput input = ShotInput.builder()
 *     .shooterPositionMeters(1.0, 2.0, 0.5)
 *     .shooterYawDegrees(45)
 *     .targetPositionMeters(5.0, 5.0, 2.5)
 *     .preferHighArc(true)
 *     .build();
 * 
 * TrajectoryResult result = solver.solve(input);
 * 
 * if (result.isSuccess()) {
 *     System.out.println("Pitch: " + result.getPitchAngleDegrees());
 *     System.out.println("RPM: " + result.getRecommendedRpm());
 * }
 * </pre>
 */
@SuppressWarnings("deprecation")
public class TrajectorySolver {
    
    /**
     * Configuration for the solver.
     */
    public static class SolverConfig {
        private final double minPitchDegrees;
        private final double maxPitchDegrees;
        private final double minRpm;
        private final double maxRpm;
        
        private final double rpmTolerance;
        private final double angleTolerance;
        
        private final FlywheelGenerator.GenerationParams flywheelGenParams;
        
        private final double crtRpmResolution;
        private final double crtAngleResolution;
        private final int crtControlLoopMs;
        private final int crtEncoderTicks;
        
        /**
         * Multiplier for target radius when checking if trajectory "hits" the target.
         * Default is 5.0 to account for large hoops/baskets - the ball just needs to
         * pass through, not hit a precise point.
         */
        private final double hoopToleranceMultiplier;
        
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
        }

        public double getMinPitchDegrees() { return minPitchDegrees; }
        public double getMaxPitchDegrees() { return maxPitchDegrees; }
        public double getMinRpm() { return minRpm; }
        public double getMaxRpm() { return maxRpm; }
        public double getRpmTolerance() { return rpmTolerance; }
        public double getAngleTolerance() { return angleTolerance; }
        public FlywheelGenerator.GenerationParams getFlywheelGenParams() { return flywheelGenParams; }
        public double getCrtRpmResolution() { return crtRpmResolution; }
        public double getCrtAngleResolution() { return crtAngleResolution; }
        public int getCrtControlLoopMs() { return crtControlLoopMs; }
        public int getCrtEncoderTicks() { return crtEncoderTicks; }
        public double getHoopToleranceMultiplier() { return hoopToleranceMultiplier; }

        public static Builder builder() {
            return new Builder();
        }

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
                .hoopToleranceMultiplier(hoopToleranceMultiplier);
        }

        public static class Builder {
            private double minPitchDegrees = 0;
            private double maxPitchDegrees = 90;
            private double minRpm = 0;
            private double maxRpm = 10_000;
            
            private double rpmTolerance = 100;
            private double angleTolerance = 1.0;
            
            private FlywheelGenerator.GenerationParams flywheelGenParams = 
                FlywheelGenerator.GenerationParams.defaultParams();
            
            private double crtRpmResolution = 1.0;
            private double crtAngleResolution = 0.1;
            private int crtControlLoopMs = 20;
            private int crtEncoderTicks = 4096;
            private double hoopToleranceMultiplier = 5.0; // Default: generous tolerance for hoops
            
            public Builder minPitchDegrees(double val) { this.minPitchDegrees = val; return this; }
            public Builder maxPitchDegrees(double val) { this.maxPitchDegrees = val; return this; }
            public Builder minRpm(double val) { this.minRpm = val; return this; }
            public Builder maxRpm(double val) { this.maxRpm = val; return this; }
            public Builder rpmTolerance(double val) { this.rpmTolerance = val; return this; }
            public Builder angleTolerance(double val) { this.angleTolerance = val; return this; }
            public Builder flywheelGenParams(FlywheelGenerator.GenerationParams val) { this.flywheelGenParams = val; return this; }
            public Builder crtRpmResolution(double val) { this.crtRpmResolution = val; return this; }
            public Builder crtAngleResolution(double val) { this.crtAngleResolution = val; return this; }
            public Builder crtControlLoopMs(int val) { this.crtControlLoopMs = val; return this; }
            public Builder crtEncoderTicks(int val) { this.crtEncoderTicks = val; return this; }
            public Builder hoopToleranceMultiplier(double val) { this.hoopToleranceMultiplier = val; return this; }
            
            public SolverConfig build() {
                return new SolverConfig(this);
            }
        }
        
        public static SolverConfig defaults() {
            return builder().build();
        }
        
        public static SolverConfig highAccuracy() {
            return builder()
                .angleTolerance(0.5)
                .rpmTolerance(50)
                .flywheelGenParams(FlywheelGenerator.GenerationParams.detailed())
                .crtAngleResolution(0.05)
                .build();
        }
        
        public static SolverConfig quickSolve() {
            return builder()
                .angleTolerance(2.0)
                .rpmTolerance(200)
                .flywheelGenParams(FlywheelGenerator.GenerationParams.quickScan())
                .build();
        }
    }
    
    private final GamePiece gamePiece;
    private final SolverConfig config;
    private final ProjectileMotion projectileMotion;
    private final FlywheelGenerator flywheelGenerator;
    
    private FlywheelConfig cachedFlywheel;
    
    public static Builder builder() {
        return new Builder();
    }
    
    public static class Builder {
        private GamePiece gamePiece = GamePieces.getCurrent();
        private SolverConfig config = SolverConfig.defaults();
        
        public Builder gamePiece(GamePiece val) { this.gamePiece = val; return this; }
        public Builder config(SolverConfig val) { this.config = val; return this; }
        
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
        this.projectileMotion = new ProjectileMotion(airResistance);
        this.flywheelGenerator = new FlywheelGenerator(gamePiece, config.getFlywheelGenParams());
    }
    
    /**
     * Creates a solver optimized for 2026 REBUILT game.
     * @return A new TrajectorySolver configured for 2026 REBUILT
     */
    public static TrajectorySolver forGame2026() {
        return forGamePiece(GamePieces.REBUILT_2026_BALL);
    }
    
    /**
     * Creates a solver for a specific game year.
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
     * @param gamePiece The game piece to use
     * @return A new TrajectorySolver configured for that game piece
     */
    public static TrajectorySolver forGamePiece(GamePiece gamePiece) {
        return new TrajectorySolver(gamePiece);
    }
    
    /**
     * Solves for the optimal trajectory given shot input.
     * 
     * This is the main entry point for the API.
     * 
     * @param input Shot input parameters
     * @return Trajectory result with recommended settings
     */
    public TrajectoryResult solve(ShotInput input) {
        if (input == null) {
            return TrajectoryResult.failure(
                TrajectoryResult.Status.INVALID_INPUT,
                "Shot input cannot be null",
                input
            );
        }
        
        // Initial target variables 
        double effectiveTargetX = input.getTargetX();
        double effectiveTargetY = input.getTargetY();
        double distance = input.getHorizontalDistanceMeters();
        
        // compensation for robot velocity
        boolean moving = Math.abs(input.getRobotVx()) > SolverConstants.getMovementThresholdMps() 
                      || Math.abs(input.getRobotVy()) > SolverConstants.getMovementThresholdMps();
        int iterations = moving ? SolverConstants.getMovingIterations() : SolverConstants.getStationaryIterations();
        double estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
        
        for (int i = 0; i < iterations; i++) {
            if (moving) {
                effectiveTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                effectiveTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
                double dx = effectiveTargetX - input.getShooterX();
                double dy = effectiveTargetY - input.getShooterY();
                distance = Math.sqrt(dx * dx + dy * dy);
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
        
        double minVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff);
        
        double targetVelocity = minVelocity * SolverConstants.getVelocityBufferMultiplier();
        
        FlywheelGenerator.GenerationResult genResult;
        if (cachedFlywheel != null) {
            FlywheelSimulator simulator = new FlywheelSimulator(cachedFlywheel, gamePiece);
            FlywheelSimulator.SimulationResult simResult = simulator.simulateForVelocity(targetVelocity);
            
            if (simResult.isAchievable) {
                genResult = new FlywheelGenerator.GenerationResult(
                    List.of(new FlywheelGenerator.ScoredConfig(
                        cachedFlywheel, simResult, 
                        simulator.scoreConfiguration(targetVelocity)
                    )), 1
                );
            } else {
                cachedFlywheel = null;
                genResult = flywheelGenerator.generateAndEvaluate(targetVelocity);
            }
        } else {
            genResult = flywheelGenerator.generateAndEvaluate(targetVelocity);
        }
        
        if (genResult.achievableCount == 0) {
            genResult = flywheelGenerator.evaluatePresets(targetVelocity);
        }
        
        if (genResult.achievableCount == 0) {
            return TrajectoryResult.failure(
                TrajectoryResult.Status.VELOCITY_EXCEEDED,
                String.format("No flywheel can achieve required velocity: %.2f m/s", targetVelocity),
                input
            );
        }
        
        FlywheelGenerator.ScoredConfig bestFlywheel = genResult.bestConfig;
        FlywheelConfig flywheel = bestFlywheel.config;
        FlywheelSimulator.SimulationResult flywheelSim = bestFlywheel.simulation;
        
        cachedFlywheel = flywheel;
        
        double actualVelocity = flywheelSim.exitVelocityMps;
        
        // Final solve loop to converge TOF and Angles with precise velocity
        double pitchAngle = 0;
        ProjectileMotion.TrajectoryResult trajSim = null;
        
        for (int i = 0; i < iterations; i++) {
            if (moving) {
                effectiveTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                effectiveTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
            }
            
            pitchAngle = projectileMotion.solveForAngle(
                gamePiece,
                input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                actualVelocity,
                effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                input.getTargetRadius(),
                input.isPreferHighArc(),
                flywheelSim.ballSpinRpm
            );
            
            if (Double.isNaN(pitchAngle)) break;
            
            // Recalculate yaw required for this compensated target
            double dx = effectiveTargetX - input.getShooterX();
            double dy = effectiveTargetY - input.getShooterY();
            double requiredYaw = Math.atan2(dy, dx);
            
            trajSim = projectileMotion.simulate(
                gamePiece,
                input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                flywheelSim.ballSpinRpm,
                actualVelocity, pitchAngle, requiredYaw,
                effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                input.getTargetRadius()
            );
            
            estimatedTof = trajSim.flightTime;
        }
        
        if (Double.isNaN(pitchAngle)) {
            return TrajectoryResult.failure(
                TrajectoryResult.Status.OUT_OF_RANGE,
                "No trajectory solution exists for given distance and velocity",
                input
            );
        }
        
        // Use pitch limits from ShotInput if provided, otherwise fall back to config
        double effectiveMinPitch = input.getMinPitchDegrees();
        double effectiveMaxPitch = input.getMaxPitchDegrees();
        
        double pitchDegrees = Math.toDegrees(pitchAngle);
        if (pitchDegrees < effectiveMinPitch || pitchDegrees > effectiveMaxPitch) {
            return TrajectoryResult.failure(
                TrajectoryResult.Status.ANGLE_EXCEEDED,
                String.format("Required pitch %.1f° outside limits [%.1f°, %.1f°]",
                    pitchDegrees, effectiveMinPitch, effectiveMaxPitch),
                input
            );
        }
        
        // Validate minimum arc height if specified
        double minArcHeight = input.getMinArcHeightMeters();
        if (minArcHeight > 0 && trajSim != null) {
            double requiredApexHeight = input.getTargetZ() + minArcHeight;
            if (trajSim.maxHeight < requiredApexHeight) {
                return TrajectoryResult.failure(
                    TrajectoryResult.Status.OUT_OF_RANGE,
                    String.format("Trajectory apex %.2fm is below required height %.2fm (target + %.2fm)",
                        trajSim.maxHeight, requiredApexHeight, minArcHeight),
                    input
                );
            }
        }
        
        // Validate that trajectory actually reaches the target
        // Use configurable hoop tolerance - the ball doesn't need to hit a precise point,
        // it just needs to pass through a hoop/basket opening
        double hoopTolerance = input.getTargetRadius() * config.getHoopToleranceMultiplier();
        if (trajSim != null && !trajSim.hitTarget && trajSim.closestApproach > hoopTolerance) {
            return TrajectoryResult.failure(
                TrajectoryResult.Status.OUT_OF_RANGE,
                String.format("Trajectory misses target by %.2fm (closest approach), tolerance is %.2fm", 
                    trajSim.closestApproach, hoopTolerance),
                input
            );
        }
        
        // Calculate Yaw Adjustment based on effective target
        double dx = effectiveTargetX - input.getShooterX();
        double dy = effectiveTargetY - input.getShooterY();
        double requiredYaw = Math.atan2(dy, dx);
        double yawAdjustment = requiredYaw - input.getShooterYaw();
        // Normalize
        while (yawAdjustment > Math.PI) yawAdjustment -= 2 * Math.PI;
        while (yawAdjustment < -Math.PI) yawAdjustment += 2 * Math.PI;
        
        double requiredRpm = flywheelSim.requiredWheelRpm;
        double timeOfFlight = trajSim.flightTime;
        double maxHeight = trajSim.maxHeight;
        
        double marginOfError = trajSim.closestApproach;
        
        TrajectoryResult.DiscreteShot discreteSolution = new TrajectoryResult.DiscreteShot(
            requiredRpm, pitchDegrees,
            (int)(requiredRpm / config.getCrtRpmResolution()),
            (int)(pitchDegrees / config.getCrtAngleResolution()),
            50.0
        );
        
        double confidence = calculateConfidence(
            bestFlywheel.score, 
            trajSim.hitTarget, 
            marginOfError, 
            input.getTargetRadius(),
            discreteSolution.score
        );
        
        return new TrajectoryResult(
            input, gamePiece,
            pitchAngle, yawAdjustment, actualVelocity,
            flywheel, flywheelSim, requiredRpm,
            timeOfFlight, maxHeight, marginOfError,
            discreteSolution, confidence
        );
    }
    
    /**
     * Solves for multiple velocities to show trajectory options.
     * Generates solutions across a range of possible velocities.
     * 
     * @param input Shot input
     * @param velocitySteps Number of velocity steps to try
     * @return Array of trajectory results at different velocities
     */
    public TrajectoryResult[] solveRange(ShotInput input, int velocitySteps) {
        double distance = input.getHorizontalDistanceMeters();
        double heightDiff = input.getHeightDifferenceMeters();
        
        double minVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff);
        double maxVelocity = minVelocity * SolverConstants.getMaxVelocityRangeMultiplier();
        
        TrajectoryResult[] results = new TrajectoryResult[velocitySteps];
        
        FlywheelGenerator.GenerationResult genResult = 
            flywheelGenerator.generateForVelocityRange(
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
            simResult.ballSpinRpm,
            velocity, pitchRadians, input.getRequiredYawRadians(),
            input.getTargetX(), input.getTargetY(), input.getTargetZ(),
            input.getTargetRadius()
        );
        
        double confidence = trajSim.hitTarget ? SolverConstants.getConfidenceHitScore() : 
            Math.max(0, SolverConstants.getConfidenceMissBase() - trajSim.closestApproach * SolverConstants.getConfidenceMissMultiplier());
        
        TrajectoryResult.DiscreteShot discreteSolution = 
            new TrajectoryResult.DiscreteShot(
                rpm, pitchDegrees,
                (int)(rpm / config.getCrtRpmResolution()),
                (int)(pitchDegrees / config.getCrtAngleResolution()),
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
     * Finds all possible shot candidates for the given input.
     * Returns candidates sorted by confidence (best first).
     * 
     * This is O(n) where n = (maxAngle - minAngle) / angleStep.
     * Default settings give ~80 iterations max.
     * 
     * @param input Shot input parameters with angle/velocity ranges
     * @return List of shot candidates sorted by confidence
     */
    public ShotCandidateList findAllCandidates(ShotInput input) {
        if (input == null) {
            return ShotCandidateList.empty(input, "Shot input cannot be null");
        }
        
        double distance = input.getHorizontalDistanceMeters();
        if (distance < SolverConstants.getMinTargetDistanceMeters()) {
            return ShotCandidateList.empty(input, 
                String.format("Target too close (< %.2fm)", SolverConstants.getMinTargetDistanceMeters()));
        }
        
        double heightDiff = input.getHeightDifferenceMeters();
        double yawAngle = input.getRequiredYawRadians();
        
        // Calculate minimum velocity needed
        double minRequiredVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff);
        double effectiveMinVelocity = Math.max(input.getMinVelocityMps(), 
            minRequiredVelocity * SolverConstants.getMinVelocityRangeMultiplier());
        double effectiveMaxVelocity = input.getMaxVelocityMps();
        
        if (effectiveMinVelocity > effectiveMaxVelocity) {
            return ShotCandidateList.empty(input, 
                String.format("Required velocity %.1f m/s exceeds maximum %.1f m/s", 
                    effectiveMinVelocity, effectiveMaxVelocity));
        }
        
        // Find all angle evaluations
        ProjectileMotion.AngleEvaluation[] evaluations = projectileMotion.findAllAnglesWithVelocity(
            gamePiece,
            input.getShooterX(), input.getShooterY(), input.getShooterZ(),
            input.getTargetX(), input.getTargetY(), input.getTargetZ(),
            input.getTargetRadius(), 0, // No spin for initial calculation
            input.getMinPitchDegrees(), input.getMaxPitchDegrees(), input.getAngleStepDegrees(),
            effectiveMinVelocity, effectiveMaxVelocity
        );
        
        if (evaluations.length == 0) {
            return ShotCandidateList.empty(input, "No valid trajectories found in angle/velocity range");
        }
        
        // Convert evaluations to candidates with scoring
        java.util.List<ShotCandidate> candidates = new java.util.ArrayList<>();
        
        // Find ranges for normalization
        double maxTof = 0, minTof = Double.MAX_VALUE;
        double maxHeight = 0;
        
        for (ProjectileMotion.AngleEvaluation eval : evaluations) {
            if (eval.timeOfFlight > maxTof) maxTof = eval.timeOfFlight;
            if (eval.timeOfFlight < minTof && eval.timeOfFlight > 0) minTof = eval.timeOfFlight;
            if (eval.maxHeight > maxHeight) maxHeight = eval.maxHeight;
        }
        
        double tofRange = maxTof - minTof;
        if (tofRange < SolverConstants.getMinTofRangeSeconds()) tofRange = 1; // Prevent division by zero
        
        for (ProjectileMotion.AngleEvaluation eval : evaluations) {
            ShotCandidate candidate = buildCandidate(eval, input, yawAngle, minTof, tofRange, maxHeight);
            candidates.add(candidate);
        }
        
        // Limit to maxCandidates
        if (candidates.size() > input.getMaxCandidates()) {
            // Sort first, then truncate
            java.util.Collections.sort(candidates);
            candidates = candidates.subList(0, input.getMaxCandidates());
        }
        
        return new ShotCandidateList(candidates, input);
    }
    
    /**
     * Builds a ShotCandidate from an angle evaluation with proper scoring.
     */
    private ShotCandidate buildCandidate(ProjectileMotion.AngleEvaluation eval, ShotInput input,
                                          double yawAngle, double minTof, double tofRange, double maxHeight) {
        // Calculate scores
        double accuracyScore = calculateAccuracyScore(eval, input.getTargetRadius());
        double stabilityScore = calculateStabilityScore(eval);
        double speedScore = calculateSpeedScore(eval, minTof, tofRange);
        double clearanceScore = calculateClearanceScore(eval, maxHeight);
        
        // Determine arc type
        ShotCandidate.ArcType arcType = determineArcType(eval.getPitchDegrees());
        
        return ShotCandidate.builder()
            .pitchAngleRadians(eval.pitchRadians)
            .requiredVelocityMps(eval.velocity)
            .yawAngleRadians(yawAngle)
            .timeOfFlightSeconds(eval.timeOfFlight)
            .maxHeightMeters(eval.maxHeight)
            .closestApproachMeters(eval.closestApproach)
            .hitsTarget(eval.hitsTarget)
            .arcType(arcType)
            .accuracyScore(accuracyScore)
            .stabilityScore(stabilityScore)
            .speedScore(speedScore)
            .clearanceScore(clearanceScore)
            .build();
    }
    
    /**
     * Calculates accuracy score based on how close trajectory comes to target center.
     */
    private double calculateAccuracyScore(ProjectileMotion.AngleEvaluation eval, double targetRadius) {
        if (eval.hitsTarget) {
            // Score based on how centered the hit is
            double relativeApproach = eval.closestApproach / targetRadius;
            return Math.max(SolverConstants.getAccuracyScoreHitMin(), 
                SolverConstants.getAccuracyScoreMax() - relativeApproach * SolverConstants.getAccuracyScoreHitMultiplier());
        } else {
            // Partial score for near misses
            double relativeError = eval.closestApproach / targetRadius;
            return Math.max(0, SolverConstants.getAccuracyScoreMissBase() 
                - (relativeError - 1) * SolverConstants.getAccuracyScoreMissMultiplier());
        }
    }
    
    /**
     * Calculates stability score based on angle sensitivity.
     * Mid-range angles (~30-50°) are typically more stable.
     */
    private double calculateStabilityScore(ProjectileMotion.AngleEvaluation eval) {
        double angleDeg = eval.getPitchDegrees();
        
        // Peak stability around optimal angle
        double optimalAngle = SolverConstants.getStabilityOptimalAngleDegrees();
        double deviation = Math.abs(angleDeg - optimalAngle);
        
        // Steep angles and very flat angles are less stable
        if (angleDeg > SolverConstants.getStabilityMaxAngleDegrees() 
                || angleDeg < SolverConstants.getStabilityMinAngleDegrees()) {
            return Math.max(SolverConstants.getStabilityUnstableMin(), 
                SolverConstants.getStabilityUnstableBase() - deviation);
        }
        
        return Math.max(SolverConstants.getStabilityStableMin(), 
            SolverConstants.getStabilityStableBase() - deviation);
    }
    
    /**
     * Calculates speed score based on time of flight.
     * Faster (lower TOF) = higher score.
     */
    private double calculateSpeedScore(ProjectileMotion.AngleEvaluation eval, double minTof, double tofRange) {
        if (tofRange < SolverConstants.getMinTofRangeSeconds()) {
            return SolverConstants.getSpeedScoreDefault(); // Default if all TOFs are similar
        }
        
        double normalizedTof = (eval.timeOfFlight - minTof) / tofRange;
        // Invert: lower TOF = higher score
        return SolverConstants.getSpeedScoreMax() - (normalizedTof * SolverConstants.getSpeedScoreRange());
    }
    
    /**
     * Calculates clearance score based on maximum height.
     * Higher trajectories get better clearance scores for obstacle avoidance.
     */
    private double calculateClearanceScore(ProjectileMotion.AngleEvaluation eval, double maxHeight) {
        if (maxHeight < SolverConstants.getClearanceMinHeightMeters()) {
            return SolverConstants.getClearanceScoreDefault(); // Default
        }
        
        double relativeHeight = eval.maxHeight / maxHeight;
        // Higher = better clearance
        return SolverConstants.getClearanceScoreBase() + (relativeHeight * SolverConstants.getClearanceScoreRange());
    }
    
    /**
     * Determines arc type based on pitch angle.
     */
    private ShotCandidate.ArcType determineArcType(double pitchDegrees) {
        if (pitchDegrees < SolverConstants.getArcTypeLowMaxDegrees()) {
            return ShotCandidate.ArcType.LOW_ARC;
        } else if (pitchDegrees > SolverConstants.getArcTypeHighMinDegrees()) {
            return ShotCandidate.ArcType.HIGH_ARC;
        } else if (pitchDegrees >= SolverConstants.getArcTypeOptimalMinDegrees() 
                && pitchDegrees <= SolverConstants.getArcTypeOptimalMaxDegrees()) {
            return ShotCandidate.ArcType.OPTIMAL_RANGE;
        } else {
            return ShotCandidate.ArcType.INTERMEDIATE;
        }
    }
    
    /**
     * Gets the best shot candidate according to the input's preference.
     * This is a convenience method that combines findAllCandidates with selection.
     * 
     * @param input Shot input with preference setting
     * @return Best candidate according to preference, or empty if none found
     */
    public java.util.Optional<ShotCandidate> getBestCandidate(ShotInput input) {
        ShotCandidateList candidates = findAllCandidates(input);
        
        if (!candidates.hasValidSolution()) {
            return java.util.Optional.empty();
        }
        
        switch (input.getShotPreference()) {
            case FASTEST:
                return candidates.getFastest();
            case HIGH_CLEARANCE:
                return candidates.getMaxClearance();
            case MOST_STABLE:
                return candidates.getMostStable();
            case MIN_VELOCITY:
                return candidates.getMinVelocity();
            case MOST_ACCURATE:
                return candidates.getMostAccurate();
            case PREFER_HIGH_ARC:
                return candidates.getBestHighArc().or(candidates::getBest);
            case PREFER_LOW_ARC:
                return candidates.getBestLowArc().or(candidates::getBest);
            case AUTO:
            default:
                return candidates.getBestHit().or(candidates::getBest);
        }
    }
    
    /**
     * Solves for the best pitch angle given the input preferences.
     * Returns just the optimal pitch angle in radians.
     * 
     * @param input Shot input parameters
     * @return Optimal pitch angle in radians, or NaN if no solution
     */
    public double solveBestPitch(ShotInput input) {
        return getBestCandidate(input)
            .map(ShotCandidate::getPitchAngleRadians)
            .orElse(Double.NaN);
    }
    
    /**
     * Solves for the best pitch angle in degrees.
     * 
     * @param input Shot input parameters
     * @return Optimal pitch angle in degrees, or NaN if no solution
     */
    public double solveBestPitchDegrees(ShotInput input) {
        return getBestCandidate(input)
            .map(ShotCandidate::getPitchAngleDegrees)
            .orElse(Double.NaN);
    }
    
    // ==================== END NEW API ====================
    
    /**
     * Gets the optimal flywheel configuration for a velocity range.
     */
    public FlywheelConfig getOptimalFlywheel(double minVelocityMps, double maxVelocityMps) {
        FlywheelGenerator.GenerationResult result = 
            flywheelGenerator.generateForVelocityRange(minVelocityMps, maxVelocityMps);
        
        return result.bestConfig != null ? result.bestConfig.config : null;
    }
    
    /**
     * Calculates confidence score for a solution.
     */
    private double calculateConfidence(double flywheelScore, boolean hitTarget,
                                       double marginOfError, double targetRadius,
                                       double crtScore) {
        double confidence = 0;
        
        confidence += Math.min(30, flywheelScore / 5);
        
        if (hitTarget) {
            confidence += 40;
        } else {
            double relativeError = marginOfError / targetRadius;
            confidence += Math.max(0, 30 - relativeError * 20);
        }
        
        confidence += Math.min(20, crtScore / 5);
        
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
    
    // Getters
    public GamePiece getGamePiece() { return gamePiece; }
    public SolverConfig getConfig() { return config; }
    public ProjectileMotion getProjectileMotion() { return projectileMotion; }
    public FlywheelGenerator getFlywheelGenerator() { return flywheelGenerator; }
    public FlywheelConfig getCachedFlywheel() { return cachedFlywheel; }
}
