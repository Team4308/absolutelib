package ca.team4308.absolutelib.math.trajectories.physics;

/**
 * Physics constants for trajectory calculations.
 * All units are in SI 
 */
public final class PhysicsConstants {
    
    private PhysicsConstants() {
    }
    
    /// Keep same
    public static final double GRAVITY = 9.80665; // m/s^2 standard gravity 
    public static final double AIR_DENSITY = 1.204; // kg/m^3 at 20C sea level
    public static final double ATMOSPHERIC_PRESSURE = 100100.235; // Pa 

    // These are all guesses plesae tune during testing
    // Most of these values are found from CD
    public static final double SPHERE_DRAG_COEFFICIENT = 0.47;
    public static final double FOAM_BALL_DRAG_COEFFICIENT = 0.50;
    public static final double MAGNUS_COEFFICIENT = 0.25;


    // Units
    public static final double RPM_TO_RAD_PER_SEC = Math.PI / 30.0;
    
    
    // Numerical integration parameters lower the number the higher the accuracy but slower the computation
    // Ideally these should be tunable at runtime or not be a constant 
    public static final double DEFAULT_TIME_STEP = 0.001; 
    public static final double MAX_FLIGHT_TIME = 5.0;     
    public static final double CONVERGENCE_THRESHOLD = 0.001; // 1mm 
    
    // Solver parameters
    public static final int MAX_ITERATIONS = 1000; // Change incase of non convergence
    public static final double ANGLE_RESOLUTION = 0.1; // degrees
    public static final double RPM_RESOLUTION = 10.0;  // RPM 
    


    /**
     * Convert RPM to radians per second.
     */
    public static double rpmToRadPerSec(double rpm) {
        return rpm * RPM_TO_RAD_PER_SEC;
    }
    
    /**
     * Convert radians per second to RPM.
     */
    public static double radPerSecToRpm(double radPerSec) {
        return radPerSec / RPM_TO_RAD_PER_SEC;
    }

}
