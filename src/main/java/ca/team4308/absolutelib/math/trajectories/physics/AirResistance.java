package ca.team4308.absolutelib.math.trajectories.physics;

import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;

/**
 * Models air resistance (drag) for projectile motion.
 * Implements both simple drag and Magnus effect for spinning balls.
 */
public class AirResistance {
    
    private final boolean enabled;
    private final double airDensity;
    private final double dragCoefficient;
    private final boolean includeMagnusEffect;
    
    /**
     * Creates air resistance model with default parameters.
     */
    public AirResistance() {
        this(true, PhysicsConstants.AIR_DENSITY, PhysicsConstants.FOAM_BALL_DRAG_COEFFICIENT, false);
    }
    
    /**
     * Creates air resistance model with custom parameters.
     * 
     * @param enabled Whether air resistance is enabled
     * @param airDensity Air density in kg/m^3
     * @param dragCoefficient Drag coefficient (Cd)
     * @param includeMagnusEffect Whether to include Magnus effect for spinning balls
     */
    public AirResistance(boolean enabled, double airDensity, double dragCoefficient, boolean includeMagnusEffect) {
        this.enabled = enabled;
        this.airDensity = airDensity;
        this.dragCoefficient = dragCoefficient;
        this.includeMagnusEffect = includeMagnusEffect;
    }
    
    /**
     * Creates disabled air resistance model (vacuum conditions).
     */
    public static AirResistance disabled() {
        return new AirResistance(false, 0, 0, false);
    }
    
    /**
     * Creates air resistance model optimized for indoor FRC competition.
     */
    public static AirResistance indoorFRC() {
        // we should add a barometic sensor Ong frfr so we can get real time data this is tottaly going to make a huge difference
        return new AirResistance(true, 1.225, PhysicsConstants.FOAM_BALL_DRAG_COEFFICIENT, false);
    }
    
    /**
     * Creates air resistance model with Magnus effect for backspin shots.
     */
    public static AirResistance withMagnus() {
        return new AirResistance(true, PhysicsConstants.AIR_DENSITY, 
            PhysicsConstants.FOAM_BALL_DRAG_COEFFICIENT, true);
    }
    
    /**
     * Calculates the drag force vector on the projectile.
     * 
     * @param gamePiece The game piece being shot
     * @param vx Velocity in X direction (m/s)
     * @param vy Velocity in Y direction (m/s)
     * @param vz Velocity in Z direction (m/s)
     * @return Array of [Fx, Fy, Fz] drag forces in Newtons
     */
    public double[] calculateDragForce(GamePiece gamePiece, double vx, double vy, double vz) {
        if (!enabled) {
            return new double[]{0, 0, 0};
        }
        
        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        if (speed < 1e-6) {
            return new double[]{0, 0, 0};
        }
        
        double radius = gamePiece.getRadiusMeters();
        double area = Math.PI * radius * radius;
        double dragMagnitude = 0.5 * airDensity * speed * speed * dragCoefficient * area;
        double fx = -dragMagnitude * (vx / speed);
        double fy = -dragMagnitude * (vy / speed);
        double fz = -dragMagnitude * (vz / speed);
        
        return new double[]{fx, fy, fz};
    }
    
    /**
     * Calculates drag acceleration on the projectile.
     * 
     * @param gamePiece The game piece
     * @param vx Velocity X (m/s)
     * @param vy Velocity Y (m/s)
     * @param vz Velocity Z (m/s)
     * @return Array of [ax, ay, az] drag accelerations in m/s^2
     */
    public double[] calculateDragAcceleration(GamePiece gamePiece, double vx, double vy, double vz) {
        double[] force = calculateDragForce(gamePiece, vx, vy, vz);
        double mass = gamePiece.getMassKg();
        
        return new double[]{
            force[0] / mass,
            force[1] / mass,
            force[2] / mass
        };
    }
    
    /**
     * Calculates Magnus force for a spinning ball.
     * The Magnus effect causes lift perpendicular to both velocity and spin axis.
     * 
     * @param gamePiece The game piece
     * @param vx Velocity X (m/s)
     * @param vy Velocity Y (m/s) 
     * @param vz Velocity Z (m/s)
     * @param spinRpm Spin rate in RPM (positive = backspin)
     * @param spinAxisX X component of spin axis (normalized)
     * @param spinAxisY Y component of spin axis (normalized)
     * @param spinAxisZ Z component of spin axis (normalized)
     * @return Array of [Fx, Fy, Fz] Magnus forces in Newtons
     */
    public double[] calculateMagnusForce(GamePiece gamePiece, 
            double vx, double vy, double vz,
            double spinRpm, double spinAxisX, double spinAxisY, double spinAxisZ) {
        
        if (!enabled || !includeMagnusEffect || Math.abs(spinRpm) < 1.0) {
            return new double[]{0, 0, 0};
        }
        
        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        if (speed < 1e-6) {
            return new double[]{0, 0, 0};
        }
        
        double radius = gamePiece.getRadiusMeters();
        double area = Math.PI * radius * radius;
        double angularVelocity = PhysicsConstants.rpmToRadPerSec(spinRpm);
        double spinParameter = Math.abs(angularVelocity * radius / speed);
        double clMagnus = PhysicsConstants.MAGNUS_COEFFICIENT * spinParameter;
        double magnusMagnitude = 0.5 * airDensity * speed * speed * clMagnus * area;
        double crossX = spinAxisY * vz - spinAxisZ * vy;
        double crossY = spinAxisZ * vx - spinAxisX * vz;
        double crossZ = spinAxisX * vy - spinAxisY * vx;
        double crossMag = Math.sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);
        
        if (crossMag < 1e-6) {
            return new double[]{0, 0, 0};
        }
        
        return new double[]{
            magnusMagnitude * crossX / crossMag,
            magnusMagnitude * crossY / crossMag,
            magnusMagnitude * crossZ / crossMag
        };
    }
    
    /**
     * Returns the terminal velocity for a given game piece.
     * 
     * @param gamePiece The game piece
     * @return Terminal velocity in m/s
     */
    public double calculateTerminalVelocity(GamePiece gamePiece) {
        if (!enabled || dragCoefficient < 1e-6) {
            return Double.POSITIVE_INFINITY;
        }
        
        double mass = gamePiece.getMassKg();
        double radius = gamePiece.getRadiusMeters();
        double area = Math.PI * radius * radius;
        
        // v_terminal = sqrt(2 * m * g / (rho * Cd * A)) (?)
        return Math.sqrt(2 * mass * PhysicsConstants.GRAVITY / (airDensity * dragCoefficient * area));
    }
    
    public boolean isEnabled() {
        return enabled;
    }
    
    public double getAirDensity() {
        return airDensity;
    }
    
    public double getDragCoefficient() {
        return dragCoefficient;
    }
    
    public boolean isMagnusEffectEnabled() {
        return includeMagnusEffect;
    }
}
