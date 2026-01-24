package ca.team4308.absolutelib.math.trajectories.motor;

/**
 * Common FRC motors with their specifications.
 * Data sourced from manufacturer datasheets and common FRC resources.
 */
public final class FRCMotors {
    
    private FRCMotors() {
        // Prevent instantiation
    }
    
    /**
     * WCP Kraken X60 - High-performance brushless motor.
     * Primary motor choice for 2026 and beyond.
     */
    public static final MotorSpec KRAKEN_X60 = new MotorSpec(
        "Kraken X60",
        6000,           // Free speed RPM
        7.09,           // Stall torque Nm
        2.0,            // Free current A
        366,            // Stall current A
        12.0,           // Nominal voltage
        0.000025,       // Rotor inertia kg*m^2 (estimated)
        0.665           // Mass kg
    );
    
    /**
     * WCP Kraken X60 with FOC (Field Oriented Control).
     * Slightly different characteristics due to FOC mode.
     */
    public static final MotorSpec KRAKEN_X60_FOC = new MotorSpec(
        "Kraken X60 FOC",
        5800,           // Slightly lower free speed in FOC
        7.09,           // Stall torque Nm
        2.0,            // Free current A
        366,            // Stall current A
        12.0,           // Nominal voltage
        0.000025,       // Rotor inertia kg*m^2
        0.665           // Mass kg
    );
    
    /**
     * REV NEO Brushless Motor.
     */
    public static final MotorSpec NEO = new MotorSpec(
        "NEO",
        5676,           // Free speed RPM
        2.6,            // Stall torque Nm
        1.8,            // Free current A
        105,            // Stall current A
        12.0,           // Nominal voltage
        0.000015,       // Rotor inertia kg*m^2 (estimated)
        0.425           // Mass kg
    );
    
    /**
     * REV NEO Vortex - Updated NEO motor.
     */
    public static final MotorSpec NEO_VORTEX = new MotorSpec(
        "NEO Vortex",
        6784,           // Free speed RPM
        3.6,            // Stall torque Nm
        1.4,            // Free current A
        211,            // Stall current A
        12.0,           // Nominal voltage
        0.000012,       // Rotor inertia kg*m^2 (estimated)
        0.360           // Mass kg
    );
    
    /**
     * REV NEO 550 - Smaller brushless motor.
     */
    public static final MotorSpec NEO_550 = new MotorSpec(
        "NEO 550",
        11000,          // Free speed RPM
        0.97,           // Stall torque Nm
        1.4,            // Free current A
        100,            // Stall current A
        12.0,           // Nominal voltage
        0.000003,       // Rotor inertia kg*m^2 (estimated)
        0.200           // Mass kg
    );
    
    /**
     * VEX Falcon 500 (legacy, but still commonly used).
     */
    public static final MotorSpec FALCON_500 = new MotorSpec(
        "Falcon 500",
        6380,           // Free speed RPM
        4.69,           // Stall torque Nm
        1.5,            // Free current A
        257,            // Stall current A
        12.0,           // Nominal voltage
        0.000018,       // Rotor inertia kg*m^2 (estimated)
        0.545           // Mass kg
    );
    
    /**
     * CIM Motor - Classic FRC motor.
     */
    public static final MotorSpec CIM = new MotorSpec(
        "CIM",
        5330,           // Free speed RPM
        2.41,           // Stall torque Nm
        2.7,            // Free current A
        131,            // Stall current A
        12.0,           // Nominal voltage
        0.000040,       // Rotor inertia kg*m^2
        1.080           // Mass kg
    );
    
    /**
     * Mini CIM Motor.
     */
    public static final MotorSpec MINI_CIM = new MotorSpec(
        "Mini CIM",
        5840,           // Free speed RPM
        1.41,           // Stall torque Nm
        3.0,            // Free current A
        89,             // Stall current A
        12.0,           // Nominal voltage
        0.000010,       // Rotor inertia kg*m^2 (estimated)
        0.544           // Mass kg
    );
    
    /**
     * AndyMark 775pro - High speed brushed motor.
     */
    public static final MotorSpec AM_775PRO = new MotorSpec(
        "775pro",
        18730,          // Free speed RPM
        0.71,           // Stall torque Nm
        0.7,            // Free current A
        134,            // Stall current A
        12.0,           // Nominal voltage
        0.000002,       // Rotor inertia kg*m^2 (estimated)
        0.362           // Mass kg
    );
    
    /**
     * All available motors for iteration.
     */
    public static final MotorSpec[] ALL_MOTORS = {
        KRAKEN_X60,
        KRAKEN_X60_FOC,
        NEO,
        NEO_VORTEX,
        NEO_550,
        FALCON_500,
        CIM,
        MINI_CIM,
        AM_775PRO
    };
    
    /**
     * Recommended motors for shooter flywheels (high torque, good speed).
     */
    public static final MotorSpec[] SHOOTER_MOTORS = {
        KRAKEN_X60,
        KRAKEN_X60_FOC,
        NEO,
        NEO_VORTEX,
        FALCON_500
    };
    
    /**
     * Gets a motor by name (case-insensitive).
     */
    public static MotorSpec getByName(String name) {
        for (MotorSpec motor : ALL_MOTORS) {
            if (motor.getName().equalsIgnoreCase(name)) {
                return motor;
            }
        }
        return null;
    }
    
    /**
     * Gets the default motor for flywheel applications.
     */
    public static MotorSpec getDefaultShooterMotor() {
        return KRAKEN_X60;
    }
}
