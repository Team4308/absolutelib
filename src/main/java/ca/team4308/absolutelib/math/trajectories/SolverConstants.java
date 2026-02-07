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

    // ==================== Velocity ====================

    /**
     * Initial velocity estimate for time-of-flight calculations (m/s).
     * Used when compensating for robot movement.
     * Default: 15.0 m/s
     */
    private static double initialVelocityEstimateMps = 15.0;
    
    /**
     * Buffer multiplier applied to minimum required velocity.
     * Adds safety margin for achievable shots.
     * Default: 1.5 (50% buffer)
     */
    private static double velocityBufferMultiplier = 1.5;
    
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

    // ==================== Motion Compensation ====================

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

    /**
     * Number of extra convergence iterations for moving robot shots in the sweep solver.
     * More iterations = better accuracy when robot is moving quickly.
     * Default: 5
     */
    private static int movingConvergenceIterations = 5;


    /**
     * Sample interval for trajectory points (seconds).
     * Lower = more points, higher accuracy, more memory.
     * Default: 0.01s (100 Hz)
     */
    private static double trajectorySampleIntervalSeconds = 0.01;


    /**
     * Default arc bias strength when obstacles require "up and over" trajectories.
     * Applied when path crosses an obstacle and no explicit bias is set.
     * Default: 0.6
     */
    private static double defaultArcBiasStrength = 0.6;
    
    /**
     * Safety margin added above obstacle clearance height (meters).
     * Ensures trajectories clear obstacles with some room.
     * Default: 0.15m (15cm)
     */
    private static double obstacleClearanceMarginMeters = 0.15;
    
    /**
     * Minimum pitch angle to try for "up and over" shots (degrees).
     * When the path crosses an obstacle, the solver won't try angles below this.
     * Default: 35 degrees
     */
    private static double forceHighArcMinPitchDegrees = 35.0;


    /**
     * Velocity scale factor for close-range shots (under threshold distance).
     * At close range, air resistance has minimal effect so no drag compensation
     * is applied — only this buffer multiplier.
     * Default: 1.3 (30% buffer)
     */
    private static double closeRangeVelocityMultiplier = 1.3;
    
    /**
     * Distance threshold for close-range detection (meters).
     * Below this distance, close-range velocity scaling is applied.
     * Default: 3.0m
     */
    private static double closeRangeThresholdMeters = 3.0;

    // ==================== Collision ====================

    /**
     * Distance from launch position (meters) within which collision checks are
     * skipped. This allows the ball to "escape" upward when the shooter is inside
     * or very close to an obstacle's footprint (e.g., shooting from under the hub).
     * Default: 0.5m
     */
    private static double collisionGraceDistanceMeters = 0.5;

    /**
     * Multiplier to compensate for air resistance when estimating minimum velocity.
     * The analytical minimum velocity formula assumes vacuum; real balls with drag
     * need significantly more speed. This is multiplied on top of velocityBufferMultiplier.
     * Default: 1.8 (80% boost for drag)
     */
    private static double dragCompensationMultiplier = 1.8;

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
    public static int getMovingConvergenceIterations() { return movingConvergenceIterations; }
    public static double getTrajectorySampleIntervalSeconds() { return trajectorySampleIntervalSeconds; }
    public static double getDefaultArcBiasStrength() { return defaultArcBiasStrength; }
    public static double getObstacleClearanceMarginMeters() { return obstacleClearanceMarginMeters; }
    public static double getForceHighArcMinPitchDegrees() { return forceHighArcMinPitchDegrees; }
    public static double getCloseRangeVelocityMultiplier() { return closeRangeVelocityMultiplier; }
    public static double getCloseRangeThresholdMeters() { return closeRangeThresholdMeters; }
    public static double getCollisionGraceDistanceMeters() { return collisionGraceDistanceMeters; }
    public static double getDragCompensationMultiplier() { return dragCompensationMultiplier; }

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
    public static void setMovingConvergenceIterations(int value) { movingConvergenceIterations = value; }
    public static void setTrajectorySampleIntervalSeconds(double value) { trajectorySampleIntervalSeconds = value; }
    public static void setDefaultArcBiasStrength(double value) { defaultArcBiasStrength = value; }
    public static void setObstacleClearanceMarginMeters(double value) { obstacleClearanceMarginMeters = value; }
    public static void setForceHighArcMinPitchDegrees(double value) { forceHighArcMinPitchDegrees = value; }
    public static void setCloseRangeVelocityMultiplier(double value) { closeRangeVelocityMultiplier = value; }
    public static void setCloseRangeThresholdMeters(double value) { closeRangeThresholdMeters = value; }
    public static void setCollisionGraceDistanceMeters(double value) { collisionGraceDistanceMeters = value; }
    public static void setDragCompensationMultiplier(double value) { dragCompensationMultiplier = value; }
    
    /**
     * Resets all constants to their default values.
     */
    public static void resetToDefaults() {
        hoopToleranceMultiplier = 5.0;
        minTargetDistanceMeters = 0.1;
        basketDescentToleranceMultiplier = 5.0;
        initialVelocityEstimateMps = 15.0;
        velocityBufferMultiplier = 1.3;
        minVelocityRangeMultiplier = 0.9;
        maxVelocityRangeMultiplier = 2.0;
        movementThresholdMps = 0.01;
        movingIterations = 3;
        stationaryIterations = 1;
        movingConvergenceIterations = 5;
        trajectorySampleIntervalSeconds = 0.01;
        defaultArcBiasStrength = 0.6;
        obstacleClearanceMarginMeters = 0.15;
        forceHighArcMinPitchDegrees = 35.0;
        closeRangeVelocityMultiplier = 1.3;
        closeRangeThresholdMeters = 3.0;
        collisionGraceDistanceMeters = 0.5;
        dragCompensationMultiplier = 1.8;
    }
}
