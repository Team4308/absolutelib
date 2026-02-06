package ca.team4308.absolutelib.math.trajectories;

/**
 * Configurable constants for the trajectory solver system.
 * 
 * All constants can be modified at runtime to tune solver behavior.
 * Call the static setters before creating a TrajectorySolver instance,
 * or use {@link #resetToDefaults()} to restore original values.
 * 
 * <h2>Example Usage</h2>
 * <pre>
 * // Make the solver more tolerant for large hoops
 * SolverConstants.setHoopToleranceMultiplier(8.0);
 * SolverConstants.setMinTargetDistanceMeters(0.05);
 * 
 * // Create solver with modified constants
 * TrajectorySolver solver = TrajectorySolver.forGame2026();
 * </pre>
 */
public final class SolverConstants {
    
    private SolverConstants() {}
    
    // ==================== Target Detection ====================
    
    /**
     * Multiplier for target radius when determining if a trajectory "hits" the target.
     * Higher values are more lenient for hoop/basket-style targets.
     * Default: 5.0 (ball within 5x target radius counts as hit)
     */
    private static double hoopToleranceMultiplier = 5.0;
    
    /**
     * Minimum horizontal distance to target (meters).
     * Shots closer than this are rejected as "too close".
     * Default: 0.1m (10cm)
     */
    private static double minTargetDistanceMeters = 0.1;
    
    /**
     * Multiplier for target radius used in basket descent detection.
     * When ball descends and is within (targetRadius * this) horizontally, it's a hit.
     * Default: 5.0
     */
    private static double basketDescentToleranceMultiplier = 5.0;
    
    // ==================== Velocity Estimation ====================
    
    /**
     * Initial velocity estimate for time-of-flight calculations (m/s).
     * Used when compensating for robot movement.
     * Default: 15.0 m/s
     */
    private static double initialVelocityEstimateMps = 15.0;
    
    /**
     * Buffer multiplier applied to minimum required velocity.
     * Adds safety margin for achievable shots.
     * Default: 1.3 (30% buffer)
     */
    private static double velocityBufferMultiplier = 1.3;
    
    /**
     * Multiplier applied to minimum velocity for velocity range calculations.
     * Default: 0.9 (allow 10% below calculated minimum)
     */
    private static double minVelocityRangeMultiplier = 0.9;
    
    /**
     * Multiplier for maximum velocity in range calculations.
     * Default: 2.0 (allow up to 2x minimum velocity)
     */
    private static double maxVelocityRangeMultiplier = 2.0;
    
    // ==================== Movement Compensation ====================
    
    /**
     * Threshold velocity (m/s) below which robot is considered stationary.
     * Default: 0.01 m/s (1 cm/s)
     */
    private static double movementThresholdMps = 0.01;
    
    /**
     * Number of iterations for trajectory convergence when robot is moving.
     * Default: 3
     */
    private static int movingIterations = 3;
    
    /**
     * Number of iterations for trajectory convergence when robot is stationary.
     * Default: 1
     */
    private static int stationaryIterations = 1;
    
    // ==================== Angle Solver ====================
    
    /**
     * Minimum angle bound for binary search (radians).
     * Default: 0.1 rad (~5.7°)
     */
    private static double minAngleBoundRadians = 0.1;
    
    /**
     * Maximum angle bound for binary search (radians from π/2).
     * Actual max = π/2 - this value.
     * Default: 0.1 rad (max ~84.3°)
     */
    private static double maxAngleOffsetRadians = 0.1;
    
    /**
     * Angle step for fallback sweep search (radians).
     * Default: 0.05 rad (~2.9°)
     */
    private static double angleSweepStepRadians = 0.05;
    
    /**
     * Convergence threshold for angle binary search (radians).
     * Solver stops when angle range < this value.
     * Default: 0.00001 rad (~0.0006°)
     */
    private static double angleConvergenceThreshold = 0.00001;
    
    /**
     * Multiplier for low angle bound adjustment in high-arc mode.
     * Default: 0.8
     */
    private static double highArcLowBoundMultiplier = 0.8;
    
    /**
     * Multiplier for high angle bound adjustment in low-arc mode.
     * Default: 1.2
     */
    private static double lowArcHighBoundMultiplier = 1.2;
    
    /**
     * Small angle offset for high-arc maximum bound (radians from π/2).
     * Default: 0.05 rad
     */
    private static double highArcMaxOffsetRadians = 0.05;
    
    /**
     * Small angle for low-arc minimum bound (radians).
     * Default: 0.05 rad
     */
    private static double lowArcMinBoundRadians = 0.05;
    
    // ==================== Trajectory Sampling ====================
    
    /**
     * Sample interval for trajectory points (seconds).
     * Lower = more points, higher accuracy, more memory.
     * Default: 0.01s (100 Hz)
     */
    private static double trajectorySampleIntervalSeconds = 0.01;
    
    /**
     * Minimum time-of-flight range before defaulting (seconds).
     * Prevents division by zero in normalization.
     * Default: 0.001s
     */
    private static double minTofRangeSeconds = 0.001;
    
    /**
     * Near-zero threshold for velocity calculations.
     * Default: 1e-6
     */
    private static double velocityZeroThreshold = 1e-6;
    
    // ==================== Scoring: Accuracy ====================
    
    /**
     * Maximum accuracy score for a direct hit.
     * Default: 100
     */
    private static double accuracyScoreMax = 100.0;
    
    /**
     * Minimum accuracy score for a direct hit.
     * Default: 50
     */
    private static double accuracyScoreHitMin = 50.0;
    
    /**
     * Score reduction multiplier for hit distance from center.
     * Default: 50
     */
    private static double accuracyScoreHitMultiplier = 50.0;
    
    /**
     * Base score for near-misses.
     * Default: 50
     */
    private static double accuracyScoreMissBase = 50.0;
    
    /**
     * Score reduction multiplier for miss distance.
     * Default: 25
     */
    private static double accuracyScoreMissMultiplier = 25.0;
    
    // ==================== Scoring: Stability ====================
    
    /**
     * Optimal angle for stability (degrees).
     * Default: 40°
     */
    private static double stabilityOptimalAngleDegrees = 40.0;
    
    /**
     * Maximum stable angle (degrees). Above this is considered unstable.
     * Default: 75°
     */
    private static double stabilityMaxAngleDegrees = 75.0;
    
    /**
     * Minimum stable angle (degrees). Below this is considered unstable.
     * Default: 10°
     */
    private static double stabilityMinAngleDegrees = 10.0;
    
    /**
     * Base stability score for unstable angles.
     * Default: 60
     */
    private static double stabilityUnstableBase = 60.0;
    
    /**
     * Minimum stability score for unstable angles.
     * Default: 20
     */
    private static double stabilityUnstableMin = 20.0;
    
    /**
     * Base stability score for stable angles.
     * Default: 90
     */
    private static double stabilityStableBase = 90.0;
    
    /**
     * Minimum stability score for stable angles.
     * Default: 30
     */
    private static double stabilityStableMin = 30.0;
    
    // ==================== Scoring: Speed ====================
    
    /**
     * Maximum speed score (for fastest trajectories).
     * Default: 95
     */
    private static double speedScoreMax = 95.0;
    
    /**
     * Speed score reduction range.
     * Default: 60
     */
    private static double speedScoreRange = 60.0;
    
    /**
     * Default speed score when TOF range is too small.
     * Default: 70
     */
    private static double speedScoreDefault = 70.0;
    
    // ==================== Scoring: Clearance ====================
    
    /**
     * Base clearance score.
     * Default: 40
     */
    private static double clearanceScoreBase = 40.0;
    
    /**
     * Clearance score range (added based on relative height).
     * Default: 55
     */
    private static double clearanceScoreRange = 55.0;
    
    /**
     * Default clearance score when max height is too small.
     * Default: 50
     */
    private static double clearanceScoreDefault = 50.0;
    
    /**
     * Minimum height threshold for clearance scoring (meters).
     * Default: 0.1m
     */
    private static double clearanceMinHeightMeters = 0.1;
    
    // ==================== Scoring: Confidence ====================
    
    /**
     * Confidence score for a confirmed hit.
     * Default: 90
     */
    private static double confidenceHitScore = 90.0;
    
    /**
     * Base confidence score for near-misses.
     * Default: 70
     */
    private static double confidenceMissBase = 70.0;
    
    /**
     * Confidence reduction multiplier per meter of miss distance.
     * Default: 100
     */
    private static double confidenceMissMultiplier = 100.0;
    
    // ==================== Arc Type Classification ====================
    
    /**
     * Maximum angle for LOW_ARC classification (degrees).
     * Default: 25°
     */
    private static double arcTypeLowMaxDegrees = 25.0;
    
    /**
     * Minimum angle for HIGH_ARC classification (degrees).
     * Default: 55°
     */
    private static double arcTypeHighMinDegrees = 55.0;
    
    /**
     * Minimum angle for OPTIMAL_RANGE classification (degrees).
     * Default: 40°
     */
    private static double arcTypeOptimalMinDegrees = 40.0;
    
    /**
     * Maximum angle for OPTIMAL_RANGE classification (degrees).
     * Default: 50°
     */
    private static double arcTypeOptimalMaxDegrees = 50.0;
    
    // ==================== Getters ====================
    
    public static double getHoopToleranceMultiplier() { return hoopToleranceMultiplier; }
    public static double getMinTargetDistanceMeters() { return minTargetDistanceMeters; }
    public static double getBasketDescentToleranceMultiplier() { return basketDescentToleranceMultiplier; }
    public static double getInitialVelocityEstimateMps() { return initialVelocityEstimateMps; }
    public static double getVelocityBufferMultiplier() { return velocityBufferMultiplier; }
    public static double getMinVelocityRangeMultiplier() { return minVelocityRangeMultiplier; }
    public static double getMaxVelocityRangeMultiplier() { return maxVelocityRangeMultiplier; }
    public static double getMovementThresholdMps() { return movementThresholdMps; }
    public static int getMovingIterations() { return movingIterations; }
    public static int getStationaryIterations() { return stationaryIterations; }
    public static double getMinAngleBoundRadians() { return minAngleBoundRadians; }
    public static double getMaxAngleOffsetRadians() { return maxAngleOffsetRadians; }
    public static double getAngleSweepStepRadians() { return angleSweepStepRadians; }
    public static double getAngleConvergenceThreshold() { return angleConvergenceThreshold; }
    public static double getHighArcLowBoundMultiplier() { return highArcLowBoundMultiplier; }
    public static double getLowArcHighBoundMultiplier() { return lowArcHighBoundMultiplier; }
    public static double getHighArcMaxOffsetRadians() { return highArcMaxOffsetRadians; }
    public static double getLowArcMinBoundRadians() { return lowArcMinBoundRadians; }
    public static double getTrajectorySampleIntervalSeconds() { return trajectorySampleIntervalSeconds; }
    public static double getMinTofRangeSeconds() { return minTofRangeSeconds; }
    public static double getVelocityZeroThreshold() { return velocityZeroThreshold; }
    public static double getAccuracyScoreMax() { return accuracyScoreMax; }
    public static double getAccuracyScoreHitMin() { return accuracyScoreHitMin; }
    public static double getAccuracyScoreHitMultiplier() { return accuracyScoreHitMultiplier; }
    public static double getAccuracyScoreMissBase() { return accuracyScoreMissBase; }
    public static double getAccuracyScoreMissMultiplier() { return accuracyScoreMissMultiplier; }
    public static double getStabilityOptimalAngleDegrees() { return stabilityOptimalAngleDegrees; }
    public static double getStabilityMaxAngleDegrees() { return stabilityMaxAngleDegrees; }
    public static double getStabilityMinAngleDegrees() { return stabilityMinAngleDegrees; }
    public static double getStabilityUnstableBase() { return stabilityUnstableBase; }
    public static double getStabilityUnstableMin() { return stabilityUnstableMin; }
    public static double getStabilityStableBase() { return stabilityStableBase; }
    public static double getStabilityStableMin() { return stabilityStableMin; }
    public static double getSpeedScoreMax() { return speedScoreMax; }
    public static double getSpeedScoreRange() { return speedScoreRange; }
    public static double getSpeedScoreDefault() { return speedScoreDefault; }
    public static double getClearanceScoreBase() { return clearanceScoreBase; }
    public static double getClearanceScoreRange() { return clearanceScoreRange; }
    public static double getClearanceScoreDefault() { return clearanceScoreDefault; }
    public static double getClearanceMinHeightMeters() { return clearanceMinHeightMeters; }
    public static double getConfidenceHitScore() { return confidenceHitScore; }
    public static double getConfidenceMissBase() { return confidenceMissBase; }
    public static double getConfidenceMissMultiplier() { return confidenceMissMultiplier; }
    public static double getArcTypeLowMaxDegrees() { return arcTypeLowMaxDegrees; }
    public static double getArcTypeHighMinDegrees() { return arcTypeHighMinDegrees; }
    public static double getArcTypeOptimalMinDegrees() { return arcTypeOptimalMinDegrees; }
    public static double getArcTypeOptimalMaxDegrees() { return arcTypeOptimalMaxDegrees; }
    
    // ==================== Setters ====================
    
    public static void setHoopToleranceMultiplier(double value) { hoopToleranceMultiplier = value; }
    public static void setMinTargetDistanceMeters(double value) { minTargetDistanceMeters = value; }
    public static void setBasketDescentToleranceMultiplier(double value) { basketDescentToleranceMultiplier = value; }
    public static void setInitialVelocityEstimateMps(double value) { initialVelocityEstimateMps = value; }
    public static void setVelocityBufferMultiplier(double value) { velocityBufferMultiplier = value; }
    public static void setMinVelocityRangeMultiplier(double value) { minVelocityRangeMultiplier = value; }
    public static void setMaxVelocityRangeMultiplier(double value) { maxVelocityRangeMultiplier = value; }
    public static void setMovementThresholdMps(double value) { movementThresholdMps = value; }
    public static void setMovingIterations(int value) { movingIterations = value; }
    public static void setStationaryIterations(int value) { stationaryIterations = value; }
    public static void setMinAngleBoundRadians(double value) { minAngleBoundRadians = value; }
    public static void setMaxAngleOffsetRadians(double value) { maxAngleOffsetRadians = value; }
    public static void setAngleSweepStepRadians(double value) { angleSweepStepRadians = value; }
    public static void setAngleConvergenceThreshold(double value) { angleConvergenceThreshold = value; }
    public static void setHighArcLowBoundMultiplier(double value) { highArcLowBoundMultiplier = value; }
    public static void setLowArcHighBoundMultiplier(double value) { lowArcHighBoundMultiplier = value; }
    public static void setHighArcMaxOffsetRadians(double value) { highArcMaxOffsetRadians = value; }
    public static void setLowArcMinBoundRadians(double value) { lowArcMinBoundRadians = value; }
    public static void setTrajectorySampleIntervalSeconds(double value) { trajectorySampleIntervalSeconds = value; }
    public static void setMinTofRangeSeconds(double value) { minTofRangeSeconds = value; }
    public static void setVelocityZeroThreshold(double value) { velocityZeroThreshold = value; }
    public static void setAccuracyScoreMax(double value) { accuracyScoreMax = value; }
    public static void setAccuracyScoreHitMin(double value) { accuracyScoreHitMin = value; }
    public static void setAccuracyScoreHitMultiplier(double value) { accuracyScoreHitMultiplier = value; }
    public static void setAccuracyScoreMissBase(double value) { accuracyScoreMissBase = value; }
    public static void setAccuracyScoreMissMultiplier(double value) { accuracyScoreMissMultiplier = value; }
    public static void setStabilityOptimalAngleDegrees(double value) { stabilityOptimalAngleDegrees = value; }
    public static void setStabilityMaxAngleDegrees(double value) { stabilityMaxAngleDegrees = value; }
    public static void setStabilityMinAngleDegrees(double value) { stabilityMinAngleDegrees = value; }
    public static void setStabilityUnstableBase(double value) { stabilityUnstableBase = value; }
    public static void setStabilityUnstableMin(double value) { stabilityUnstableMin = value; }
    public static void setStabilityStableBase(double value) { stabilityStableBase = value; }
    public static void setStabilityStableMin(double value) { stabilityStableMin = value; }
    public static void setSpeedScoreMax(double value) { speedScoreMax = value; }
    public static void setSpeedScoreRange(double value) { speedScoreRange = value; }
    public static void setSpeedScoreDefault(double value) { speedScoreDefault = value; }
    public static void setClearanceScoreBase(double value) { clearanceScoreBase = value; }
    public static void setClearanceScoreRange(double value) { clearanceScoreRange = value; }
    public static void setClearanceScoreDefault(double value) { clearanceScoreDefault = value; }
    public static void setClearanceMinHeightMeters(double value) { clearanceMinHeightMeters = value; }
    public static void setConfidenceHitScore(double value) { confidenceHitScore = value; }
    public static void setConfidenceMissBase(double value) { confidenceMissBase = value; }
    public static void setConfidenceMissMultiplier(double value) { confidenceMissMultiplier = value; }
    public static void setArcTypeLowMaxDegrees(double value) { arcTypeLowMaxDegrees = value; }
    public static void setArcTypeHighMinDegrees(double value) { arcTypeHighMinDegrees = value; }
    public static void setArcTypeOptimalMinDegrees(double value) { arcTypeOptimalMinDegrees = value; }
    public static void setArcTypeOptimalMaxDegrees(double value) { arcTypeOptimalMaxDegrees = value; }
    
    /**
     * Resets all constants to their default values.
     */
    public static void resetToDefaults() {
        // Target Detection
        hoopToleranceMultiplier = 5.0;
        minTargetDistanceMeters = 0.1;
        basketDescentToleranceMultiplier = 5.0;
        
        // Velocity Estimation
        initialVelocityEstimateMps = 15.0;
        velocityBufferMultiplier = 1.3;
        minVelocityRangeMultiplier = 0.9;
        maxVelocityRangeMultiplier = 2.0;
        
        // Movement Compensation
        movementThresholdMps = 0.01;
        movingIterations = 3;
        stationaryIterations = 1;
        
        // Angle Solver
        minAngleBoundRadians = 0.1;
        maxAngleOffsetRadians = 0.1;
        angleSweepStepRadians = 0.05;
        angleConvergenceThreshold = 0.00001;
        highArcLowBoundMultiplier = 0.8;
        lowArcHighBoundMultiplier = 1.2;
        highArcMaxOffsetRadians = 0.05;
        lowArcMinBoundRadians = 0.05;
        
        // Trajectory Sampling
        trajectorySampleIntervalSeconds = 0.01;
        minTofRangeSeconds = 0.001;
        velocityZeroThreshold = 1e-6;
        
        // Scoring: Accuracy
        accuracyScoreMax = 100.0;
        accuracyScoreHitMin = 50.0;
        accuracyScoreHitMultiplier = 50.0;
        accuracyScoreMissBase = 50.0;
        accuracyScoreMissMultiplier = 25.0;
        
        // Scoring: Stability
        stabilityOptimalAngleDegrees = 40.0;
        stabilityMaxAngleDegrees = 75.0;
        stabilityMinAngleDegrees = 10.0;
        stabilityUnstableBase = 60.0;
        stabilityUnstableMin = 20.0;
        stabilityStableBase = 90.0;
        stabilityStableMin = 30.0;
        
        // Scoring: Speed
        speedScoreMax = 95.0;
        speedScoreRange = 60.0;
        speedScoreDefault = 70.0;
        
        // Scoring: Clearance
        clearanceScoreBase = 40.0;
        clearanceScoreRange = 55.0;
        clearanceScoreDefault = 50.0;
        clearanceMinHeightMeters = 0.1;
        
        // Scoring: Confidence
        confidenceHitScore = 90.0;
        confidenceMissBase = 70.0;
        confidenceMissMultiplier = 100.0;
        
        // Arc Type Classification
        arcTypeLowMaxDegrees = 25.0;
        arcTypeHighMinDegrees = 55.0;
        arcTypeOptimalMinDegrees = 40.0;
        arcTypeOptimalMaxDegrees = 50.0;
    }
}
