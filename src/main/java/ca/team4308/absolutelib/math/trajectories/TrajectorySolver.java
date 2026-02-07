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
            private double hoopToleranceMultiplier = 5.0; 
            
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

    // ===== Internal scoring constants (not user-tunable) =====
    private static final double ACCURACY_SCORE_MAX = 100.0;
    private static final double ACCURACY_SCORE_HIT_MIN = 50.0;
    private static final double ACCURACY_HIT_MULTIPLIER = 50.0;
    private static final double ACCURACY_MISS_BASE = 50.0;
    private static final double ACCURACY_MISS_MULTIPLIER = 25.0;

    private static final double STABILITY_OPTIMAL_ANGLE_DEG = 40.0;
    private static final double STABILITY_MAX_ANGLE_DEG = 75.0;
    private static final double STABILITY_MIN_ANGLE_DEG = 10.0;
    private static final double STABILITY_UNSTABLE_BASE = 60.0;
    private static final double STABILITY_UNSTABLE_MIN = 20.0;
    private static final double STABILITY_STABLE_BASE = 90.0;
    private static final double STABILITY_STABLE_MIN = 30.0;

    private static final double SPEED_SCORE_MAX = 95.0;
    private static final double SPEED_SCORE_RANGE = 60.0;
    private static final double SPEED_SCORE_DEFAULT = 70.0;

    private static final double CLEARANCE_SCORE_BASE = 40.0;
    private static final double CLEARANCE_SCORE_RANGE = 55.0;
    private static final double CLEARANCE_SCORE_DEFAULT = 50.0;
    private static final double CLEARANCE_MIN_HEIGHT = 0.1;

    private static final double CONFIDENCE_HIT_SCORE = 90.0;
    private static final double CONFIDENCE_MISS_BASE = 70.0;
    private static final double CONFIDENCE_MISS_MULTIPLIER = 100.0;

    private static final double ARC_LOW_MAX_DEG = 25.0;
    private static final double ARC_HIGH_MIN_DEG = 55.0;
    private static final double ARC_OPTIMAL_MIN_DEG = 40.0;
    private static final double ARC_OPTIMAL_MAX_DEG = 50.0;

    private static final double MIN_TOF_RANGE = 0.001;

    /**
     * Checks if a trajectory collides with any obstacle, respecting grace distance
     * and the opening exemption for descending balls.
     */
    private static boolean trajectoryCollides(ProjectileMotion.TrajectoryResult trajSim,
            ShotInput input, double shooterX, double shooterY) {
        if (!input.isCollisionCheckEnabled() || trajSim.trajectory.length == 0) return false;
        
        double graceDistance = SolverConstants.getCollisionGraceDistanceMeters();
        double graceDist2 = graceDistance * graceDistance;
        
        for (ObstacleConfig obstacle : input.getObstacles()) {
            for (ProjectileMotion.TrajectoryState state : trajSim.trajectory) {
                double sdx = state.x - shooterX;
                double sdy = state.y - shooterY;
                if (sdx * sdx + sdy * sdy < graceDist2) continue;
                
                if (state.vz < 0 && obstacle.isWithinOpening(state.x, state.y)) continue;
                
                if (obstacle.checkCollision(state.x, state.y, state.z)) {
                    return true;
                }
            }
        }
        return false;
    }
    
    /**
     * Overload for findAllCandidates which uses AngleEvaluation trajectory data.
     */
    private static boolean evaluationCollides(ProjectileMotion.AngleEvaluation eval,
            ShotInput input, double shooterX, double shooterY) {
        if (!input.isCollisionCheckEnabled() || eval.trajectory == null) return false;
        
        double graceDistance = SolverConstants.getCollisionGraceDistanceMeters();
        double graceDist2 = graceDistance * graceDistance;
        
        for (ObstacleConfig obstacle : input.getObstacles()) {
            for (ProjectileMotion.TrajectoryState state : eval.trajectory.trajectory) {
                double sdx = state.x - shooterX;
                double sdy = state.y - shooterY;
                if (sdx * sdx + sdy * sdy < graceDist2) continue;
                
                if (state.vz < 0 && obstacle.isWithinOpening(state.x, state.y)) continue;
                
                if (obstacle.checkCollision(state.x, state.y, state.z)) {
                    return true;
                }
            }
        }
        return false;
    }
    
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
     * Supports collision-aware solving, arc bias, full pitch range search,
     * and robust motion compensation.
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
        
        // Motion
        boolean moving = Math.abs(input.getRobotVx()) > SolverConstants.getMovementThresholdMps()
                      || Math.abs(input.getRobotVy()) > SolverConstants.getMovementThresholdMps();
        int convergenceIterations = moving
            ? SolverConstants.getMovingConvergenceIterations()
            : SolverConstants.getStationaryIterations();
        
        double distance = input.getHorizontalDistanceMeters();
        double estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
        
        double effectiveTargetX = input.getTargetX();
        double effectiveTargetY = input.getTargetY();

        for (int i = 0; i < convergenceIterations; i++) {
            if (moving) {
                effectiveTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                effectiveTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
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
        
        // Obstacle and arc requirements
        boolean pathCrossesObstacle = input.pathRequiresArc();
        double requiredClearance = input.getRequiredClearanceHeight();

    
        double effectiveMinPitch = input.getMinPitchDegrees();
        double effectiveMaxPitch = input.getMaxPitchDegrees();
        
        if (pathCrossesObstacle) {
            effectiveMinPitch = Math.max(effectiveMinPitch, SolverConstants.getForceHighArcMinPitchDegrees());
        }

        double minVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff);

        boolean isCloseRange = distance < SolverConstants.getCloseRangeThresholdMeters();
        double velocityBuffer;
        if (isCloseRange) {
            velocityBuffer = SolverConstants.getCloseRangeVelocityMultiplier();
        } else {
            velocityBuffer = SolverConstants.getVelocityBufferMultiplier()
                * SolverConstants.getDragCompensationMultiplier();
        }
        double targetVelocity = minVelocity * velocityBuffer;
        
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
        
        double bestPitchAngle = Double.NaN;
        ProjectileMotion.TrajectoryResult bestTrajSim = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        
        double angleStep = input.getAngleStepDegrees();
        
        for (double pitchDeg = effectiveMinPitch; pitchDeg <= effectiveMaxPitch; pitchDeg += angleStep) {
            double pitchRad = Math.toRadians(pitchDeg);
            
            double iterTargetX = effectiveTargetX;
            double iterTargetY = effectiveTargetY;

            if (moving) {
                iterTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                iterTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
            }
            
            double dx = iterTargetX - input.getShooterX();
            double dy = iterTargetY - input.getShooterY();
            double requiredYaw = Math.atan2(dy, dx);
            
            ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                gamePiece,
                input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                actualVelocity, pitchRad, requiredYaw,
                flywheelSim.ballSpinRpm,
                iterTargetX, iterTargetY, input.getTargetZ(),
                input.getTargetRadius()
            );
            
            if (trajSim.flightTime > 0) {
                estimatedTof = trajSim.flightTime;
            }

            if (trajectoryCollides(trajSim, input, input.getShooterX(), input.getShooterY())) continue;

            double minArcHeight = input.getMinArcHeightMeters();
            if (minArcHeight > 0) {
                double requiredApexHeight = input.getTargetZ() + minArcHeight;
                if (trajSim.maxHeight < requiredApexHeight) {
                    continue;
                }
            }

            if (requiredClearance > 0 && trajSim.maxHeight < requiredClearance) {
                continue;
            }

            double hoopTolerance = input.getTargetRadius() * config.getHoopToleranceMultiplier();
            boolean hitsTarget = trajSim.hitTarget || trajSim.closestApproach <= hoopTolerance;
            
            if (!hitsTarget) continue;

            double score = scoreCandidate(trajSim, pitchDeg, input, requiredClearance);
            
            if (score > bestScore) {
                bestScore = score;
                bestPitchAngle = pitchRad;
                bestTrajSim = trajSim;
            }
        }
        // Never trust lingfeng 
        if (Double.isNaN(bestPitchAngle) || bestTrajSim == null) {
            String reason = pathCrossesObstacle 
                ? "No collision-free trajectory found; all angles hit obstacles or miss target"
                : "No trajectory solution exists for given distance and velocity";
            return TrajectoryResult.failure(
                TrajectoryResult.Status.OUT_OF_RANGE,
                reason,
                input
            );
        }
        
        double pitchDegrees = Math.toDegrees(bestPitchAngle);
        

        double dx = effectiveTargetX - input.getShooterX();
        double dy = effectiveTargetY - input.getShooterY();
        double requiredYaw = Math.atan2(dy, dx);
        double yawAdjustment = requiredYaw - input.getShooterYaw();
        while (yawAdjustment > Math.PI) yawAdjustment -= 2 * Math.PI;
        while (yawAdjustment < -Math.PI) yawAdjustment += 2 * Math.PI;
        
        double requiredRpm = flywheelSim.requiredWheelRpm;
        double timeOfFlight = bestTrajSim.flightTime;
        double maxHeight = bestTrajSim.maxHeight;
        double marginOfError = bestTrajSim.closestApproach;
        
        TrajectoryResult.DiscreteShot discreteSolution = new TrajectoryResult.DiscreteShot(
            requiredRpm, pitchDegrees,
            (int)(requiredRpm / config.getCrtRpmResolution()),
            (int)(pitchDegrees / config.getCrtAngleResolution()),
            50.0
        );
        
        double confidence = calculateConfidence(
            bestFlywheel.score, 
            bestTrajSim.hitTarget, 
            marginOfError, 
            input.getTargetRadius(),
            discreteSolution.score
        );
        
        return new TrajectoryResult(
            input, gamePiece,
            bestPitchAngle, yawAdjustment, actualVelocity,
            flywheel, flywheelSim, requiredRpm,
            timeOfFlight, maxHeight, marginOfError,
            discreteSolution, confidence
        );
    }
    
    /**
     * Scores a trajectory candidate based on accuracy, arc preference, 
     * obstacle clearance, and stability.
     * Higher score = better candidate.
     */
    private double scoreCandidate(ProjectileMotion.TrajectoryResult traj, double pitchDeg,
                                   ShotInput input, double requiredClearance) {
        double score = 0;

        double relativeApproach = traj.closestApproach / Math.max(0.01, input.getTargetRadius());
        score += Math.max(0, 40 - relativeApproach * 20);

        if (traj.hitTarget) {
            score += 15;
        }

        double optimalAngle = STABILITY_OPTIMAL_ANGLE_DEG;
        double angleDeviation = Math.abs(pitchDeg - optimalAngle);
        score += Math.max(0, 10 - angleDeviation * 0.2);

        score += Math.max(0, 20 - traj.flightTime * 5);
        
        // clearance 
        if (requiredClearance > 0 && traj.maxHeight > requiredClearance) {
            double clearanceMargin = traj.maxHeight - requiredClearance;
            score += Math.min(10, clearanceMargin * 10);
        }
        
        double preferredArc = input.getPreferredArcHeightMeters();
        double biasStrength = input.getArcBiasStrength();

        if (preferredArc > 0 && biasStrength > 0) {
            double arcDeviation = Math.abs(traj.maxHeight - preferredArc);
            double arcBonus = Math.max(0, 20 - arcDeviation * 10);
            score += arcBonus * biasStrength;
        } else if (input.pathRequiresArc()) {
            double autoBias = SolverConstants.getDefaultArcBiasStrength();
            double autoPreferred = requiredClearance + 0.3;
            double arcDeviation = Math.abs(traj.maxHeight - autoPreferred);
            score += Math.max(0, 15 - arcDeviation * 8) * autoBias;
        }
        
        return score;
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
        
        double minVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff)
            * SolverConstants.getDragCompensationMultiplier();
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
            velocity, pitchRadians, input.getRequiredYawRadians(),
            simResult.ballSpinRpm,
            input.getTargetX(), input.getTargetY(), input.getTargetZ(),
            input.getTargetRadius()
        );
        
        double confidence = trajSim.hitTarget ? CONFIDENCE_HIT_SCORE : 
            Math.max(0, CONFIDENCE_MISS_BASE - trajSim.closestApproach * CONFIDENCE_MISS_MULTIPLIER);
        
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
     * Solves for the best pitch angle given the current measured flywheel RPM.
     * 
     * <p>Used for RPM feedback during rapid-fire shooting. When shooting many balls
     * in succession (20-50), the flywheel slows down between shots. Instead of waiting
     * for the flywheel to spin back up to the ideal RPM, this method finds the best
     * angle to shoot at the <em>current</em> RPM — treating velocity as a fixed constraint
     * and optimizing angle only.</p>
     * 
     * <p>Typical usage pattern:</p>
     * <ol>
     *   <li>First shot: use {@link #solve(ShotInput)} for the ideal RPM + angle</li>
     *   <li>Subsequent shots: read actual flywheel RPM from encoder, call this method
     *       to get the best angle at that velocity</li>
     * </ol>
     * 
     * <pre>
     * TrajectoryResult ideal = solver.solve(input);
     * // ... shoot, flywheel slows down ...
     * double currentRpm = flywheelEncoder.getVelocity();
     * TrajectoryResult adjusted = solver.solveAtCurrentRpm(input, flywheelConfig, currentRpm);
     * if (adjusted.isSuccess()) {
     *     setPitch(adjusted.getPitchAngleDegrees());
     * }
     * </pre>
     * 
     * @param input Shot input parameters (position, target, obstacles, etc.)
     * @param flywheel The flywheel configuration to simulate with
     * @param currentRpm The current measured flywheel RPM from encoder feedback
     * @return Trajectory result with the best angle for the given RPM, or failure if
     *         the current RPM cannot reach the target at any valid angle
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
        
        boolean moving = Math.abs(input.getRobotVx()) > SolverConstants.getMovementThresholdMps()
                      || Math.abs(input.getRobotVy()) > SolverConstants.getMovementThresholdMps();
        int convergenceIterations = moving
            ? SolverConstants.getMovingConvergenceIterations()
            : SolverConstants.getStationaryIterations();
        
        double distance = input.getHorizontalDistanceMeters();
        double estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
        
        double effectiveTargetX = input.getTargetX();
        double effectiveTargetY = input.getTargetY();

        for (int i = 0; i < convergenceIterations; i++) {
            if (moving) {
                effectiveTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                effectiveTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
                double ddx = effectiveTargetX - input.getShooterX();
                double ddy = effectiveTargetY - input.getShooterY();
                distance = Math.sqrt(ddx * ddx + ddy * ddy);
                estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
            }
        }
        
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
            effectiveMinPitch = Math.max(effectiveMinPitch, SolverConstants.getForceHighArcMinPitchDegrees());
        }
        
        double bestPitchAngle = Double.NaN;
        ProjectileMotion.TrajectoryResult bestTrajSim = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        
        double angleStep = input.getAngleStepDegrees();
        
        for (double pitchDeg = effectiveMinPitch; pitchDeg <= effectiveMaxPitch; pitchDeg += angleStep) {
            double pitchRad = Math.toRadians(pitchDeg);
            
            double iterTargetX = effectiveTargetX;
            double iterTargetY = effectiveTargetY;

            if (moving) {
                iterTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                iterTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
            }
            
            double dx = iterTargetX - input.getShooterX();
            double dy = iterTargetY - input.getShooterY();
            double requiredYaw = Math.atan2(dy, dx);
            
            ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                gamePiece,
                input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                actualVelocity, pitchRad, requiredYaw,
                ballSpin,
                iterTargetX, iterTargetY, input.getTargetZ(),
                input.getTargetRadius()
            );
            
            if (trajSim.flightTime > 0) {
                estimatedTof = trajSim.flightTime;
            }

            if (trajectoryCollides(trajSim, input, input.getShooterX(), input.getShooterY())) continue;

            double minArcHeight = input.getMinArcHeightMeters();
            if (minArcHeight > 0) {
                double requiredApexHeight = input.getTargetZ() + minArcHeight;
                if (trajSim.maxHeight < requiredApexHeight) {
                    continue;
                }
            }

            if (requiredClearance > 0 && trajSim.maxHeight < requiredClearance) {
                continue;
            }

            double hoopTolerance = input.getTargetRadius() * config.getHoopToleranceMultiplier();
            boolean hitsTarget = trajSim.hitTarget || trajSim.closestApproach <= hoopTolerance;
            
            if (!hitsTarget) continue;

            double score = scoreCandidate(trajSim, pitchDeg, input, requiredClearance);
            
            if (score > bestScore) {
                bestScore = score;
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
        while (yawAdjustment > Math.PI) yawAdjustment -= 2 * Math.PI;
        while (yawAdjustment < -Math.PI) yawAdjustment += 2 * Math.PI;
        
        TrajectoryResult.DiscreteShot discreteSolution = new TrajectoryResult.DiscreteShot(
            currentRpm, pitchDegrees,
            (int)(currentRpm / config.getCrtRpmResolution()),
            (int)(pitchDegrees / config.getCrtAngleResolution()),
            50.0
        );
        
        double confidence = calculateConfidence(
            simulator.scoreConfiguration(actualVelocity),
            bestTrajSim.hitTarget,
            bestTrajSim.closestApproach,
            input.getTargetRadius(),
            discreteSolution.score
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
     * Finds all possible shot candidates for the given input.
     * Returns candidates sorted by confidence (best first).
     * 
     * Supports collision-aware filtering, motion compensation, and arc bias scoring.
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
        
        // ===== Motion compensation =====
        boolean moving = Math.abs(input.getRobotVx()) > SolverConstants.getMovementThresholdMps()
                      || Math.abs(input.getRobotVy()) > SolverConstants.getMovementThresholdMps();
        double estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
        
        double effectiveTargetX = input.getTargetX();
        double effectiveTargetY = input.getTargetY();
        
        if (moving) {
            int convergenceIters = SolverConstants.getMovingConvergenceIterations();
            for (int i = 0; i < convergenceIters; i++) {
                effectiveTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                effectiveTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
                double dx = effectiveTargetX - input.getShooterX();
                double dy = effectiveTargetY - input.getShooterY();
                distance = Math.sqrt(dx * dx + dy * dy);
                estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
            }
        }
        
        double heightDiff = input.getHeightDifferenceMeters();
        double yawAngle = Math.atan2(effectiveTargetY - input.getShooterY(), 
                                     effectiveTargetX - input.getShooterX());
        
        // ===== Obstacle analysis =====
        boolean pathCrossesObstacle = input.pathRequiresArc();
        double requiredClearance = input.getRequiredClearanceHeight();

        double effectiveMinPitch = input.getMinPitchDegrees();
        double effectiveMaxPitch = input.getMaxPitchDegrees();
        
        if (pathCrossesObstacle) {
            effectiveMinPitch = Math.max(effectiveMinPitch, SolverConstants.getForceHighArcMinPitchDegrees());
        }

        double minRequiredVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff);
        double dragComp = SolverConstants.getDragCompensationMultiplier();
        double effectiveMinVelocity = Math.max(input.getMinVelocityMps(),
            minRequiredVelocity * SolverConstants.getMinVelocityRangeMultiplier() * dragComp);
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
            effectiveTargetX, effectiveTargetY, input.getTargetZ(),
            input.getTargetRadius(), 0, // No spin for initial calculation
            effectiveMinPitch, effectiveMaxPitch, input.getAngleStepDegrees(),
            effectiveMinVelocity, effectiveMaxVelocity
        );
        
        if (evaluations.length == 0) {
            return ShotCandidateList.empty(input, "No valid trajectories found in angle/velocity range");
        }
        
        java.util.List<ShotCandidate> candidates = new java.util.ArrayList<>();

        double maxTof = 0, minTof = Double.MAX_VALUE;
        double maxHeight = 0;
        
        for (ProjectileMotion.AngleEvaluation eval : evaluations) {
            if (evaluationCollides(eval, input, input.getShooterX(), input.getShooterY())) continue;

            if (requiredClearance > 0 && eval.maxHeight < requiredClearance) {
                continue;
            }
            
            if (eval.timeOfFlight > maxTof) maxTof = eval.timeOfFlight;
            if (eval.timeOfFlight < minTof && eval.timeOfFlight > 0) minTof = eval.timeOfFlight;
            if (eval.maxHeight > maxHeight) maxHeight = eval.maxHeight;
        }
        
        double tofRange = maxTof - minTof;
        if (tofRange < MIN_TOF_RANGE) tofRange = 1;
        
        for (ProjectileMotion.AngleEvaluation eval : evaluations) {
            if (evaluationCollides(eval, input, input.getShooterX(), input.getShooterY())) continue;
            
            if (requiredClearance > 0 && eval.maxHeight < requiredClearance) {
                continue;
            }
            
            ShotCandidate candidate = buildCandidate(eval, input, yawAngle, minTof, tofRange, maxHeight);
            candidates.add(candidate);
        }
        
        if (candidates.size() > input.getMaxCandidates()) {
            java.util.Collections.sort(candidates);
            candidates = candidates.subList(0, input.getMaxCandidates());
        }
        
        if (candidates.isEmpty()) {
            String reason = pathCrossesObstacle 
                ? "All trajectories collide with obstacles or fail clearance" 
                : "No valid trajectories found in angle/velocity range";
            return ShotCandidateList.empty(input, reason);
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
            double relativeApproach = eval.closestApproach / targetRadius;
            return Math.max(ACCURACY_SCORE_HIT_MIN, 
                ACCURACY_SCORE_MAX - relativeApproach * ACCURACY_HIT_MULTIPLIER);
        } else {
            double relativeError = eval.closestApproach / targetRadius;
            return Math.max(0, ACCURACY_MISS_BASE 
                - (relativeError - 1) * ACCURACY_MISS_MULTIPLIER);
        }
    }
    
    /**
     * Calculates stability score based on angle sensitivity.
     * Mid-range angles (~30-50°) are typically more stable.
     */
    private double calculateStabilityScore(ProjectileMotion.AngleEvaluation eval) {
        double angleDeg = eval.getPitchDegrees();
        
        double optimalAngle = STABILITY_OPTIMAL_ANGLE_DEG;
        double deviation = Math.abs(angleDeg - optimalAngle);
        
        if (angleDeg > STABILITY_MAX_ANGLE_DEG
                || angleDeg < STABILITY_MIN_ANGLE_DEG) {
            return Math.max(STABILITY_UNSTABLE_MIN, 
                STABILITY_UNSTABLE_BASE - deviation);
        }
        
        return Math.max(STABILITY_STABLE_MIN, 
            STABILITY_STABLE_BASE - deviation);
    }
    
    /**
     * Calculates speed score based on time of flight.
     * Faster (lower TOF) = higher score.
     */
    private double calculateSpeedScore(ProjectileMotion.AngleEvaluation eval, double minTof, double tofRange) {
        if (tofRange < MIN_TOF_RANGE) {
            return SPEED_SCORE_DEFAULT;
        }
        
        double normalizedTof = (eval.timeOfFlight - minTof) / tofRange;
        return SPEED_SCORE_MAX - (normalizedTof * SPEED_SCORE_RANGE);
    }
    
    /**
     * Calculates clearance score based on maximum height.
     * Higher trajectories get better clearance scores for obstacle avoidance.
     */
    private double calculateClearanceScore(ProjectileMotion.AngleEvaluation eval, double maxHeight) {
        if (maxHeight < CLEARANCE_MIN_HEIGHT) {
            return CLEARANCE_SCORE_DEFAULT;
        }
        
        double relativeHeight = eval.maxHeight / maxHeight;
        return CLEARANCE_SCORE_BASE + (relativeHeight * CLEARANCE_SCORE_RANGE);
    }
    
    /**
     * Determines arc type based on pitch angle.
     */
    private ShotCandidate.ArcType determineArcType(double pitchDegrees) {
        if (pitchDegrees < ARC_LOW_MAX_DEG) {
            return ShotCandidate.ArcType.LOW_ARC;
        } else if (pitchDegrees > ARC_HIGH_MIN_DEG) {
            return ShotCandidate.ArcType.HIGH_ARC;
        } else if (pitchDegrees >= ARC_OPTIMAL_MIN_DEG 
                && pitchDegrees <= ARC_OPTIMAL_MAX_DEG) {
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
            /// God help me
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
    
    public GamePiece getGamePiece() { return gamePiece; }
    public SolverConfig getConfig() { return config; }
    public ProjectileMotion getProjectileMotion() { return projectileMotion; }
    public FlywheelGenerator getFlywheelGenerator() { return flywheelGenerator; }
    public FlywheelConfig getCachedFlywheel() { return cachedFlywheel; }
}
