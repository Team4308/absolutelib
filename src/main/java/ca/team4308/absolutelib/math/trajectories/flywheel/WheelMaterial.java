package ca.team4308.absolutelib.math.trajectories.flywheel;

/**
 * Wheel material properties affecting energy transfer and grip.
 * Durometer values use Shore A scale common for soft rubber/urethane wheels.
 */
public class WheelMaterial {
    
    private final String name;
    private final int durometer;         // Shore A hardness (0-100)
    private final double density;        // kg/m^3
    private final double frictionCoeff;  // Static friction coefficient
    private final double energyLossFactor; // Energy lost per compression cycle (0-1)
    private final double gripFactor;     // Relative grip vs slip (0-1, higher = more grip)
    
    /**
     * Creates a wheel material specification.
     * 
     * @param name Material name
     * @param durometer Shore A hardness (30-95 typical)
     * @param density Material density in kg/m^3
     * @param frictionCoeff Coefficient of friction
     * @param energyLossFactor Energy loss factor (0-1)
     * @param gripFactor Grip factor (0-1)
     */
    public WheelMaterial(String name, int durometer, double density,
                         double frictionCoeff, double energyLossFactor, double gripFactor) {
        this.name = name;
        this.durometer = durometer;
        this.density = density;
        this.frictionCoeff = frictionCoeff;
        this.energyLossFactor = energyLossFactor;
        this.gripFactor = gripFactor;
    }
    
    // Predefined materials
    
    /**
     * Soft compliant wheel (30A durometer).
     * Maximum grip but higher energy loss.
     */
    public static final WheelMaterial SOFT_COMPLIANT = new WheelMaterial(
        "Soft Compliant (30A)", 30, 1100, 1.2, 0.20, 0.95
    );
    
    /**
     * Medium soft wheel (40A durometer).
     * Good grip with moderate energy loss.
     */
    public static final WheelMaterial MEDIUM_SOFT = new WheelMaterial(
        "Medium Soft (40A)", 40, 1150, 1.1, 0.15, 0.88
    );
    
    /**
     * Standard durometer (50A).
     * Balanced performance.
     */
    public static final WheelMaterial STANDARD = new WheelMaterial(
        "Standard (50A)", 50, 1180, 1.0, 0.12, 0.82
    );
    
    /**
     * Stealth wheels / Blue nitrile (60A durometer).
     * Common FRC choice for good balance.
     */
    public static final WheelMaterial STEALTH_BLUE = new WheelMaterial(
        "Stealth Blue (60A)", 60, 1200, 0.95, 0.10, 0.78
    );
    
    /**
     * Firm grip wheel (70A durometer).
     * Lower energy loss, reduced grip.
     */
    public static final WheelMaterial FIRM_GRIP = new WheelMaterial(
        "Firm Grip (70A)", 70, 1220, 0.85, 0.08, 0.72
    );
    
    /**
     * Hard wheel (80A durometer).
     * Minimal energy loss, reduced grip.
     */
    public static final WheelMaterial HARD = new WheelMaterial(
        "Hard (80A)", 80, 1250, 0.75, 0.05, 0.65
    );
    
    /**
     * Very hard / semi-rigid (90A durometer).
     * Minimal deformation and energy loss.
     */
    public static final WheelMaterial VERY_HARD = new WheelMaterial(
        "Very Hard (90A)", 90, 1280, 0.65, 0.03, 0.55
    );
    
    /**
     * Green compliant wheel (35A) - Popular FRC choice.
     */
    public static final WheelMaterial GREEN_COMPLIANT = new WheelMaterial(
        "Green Compliant (35A)", 35, 1120, 1.15, 0.18, 0.92
    );
    
    /**
     * Colson Performa wheel material (85A).
     */
    public static final WheelMaterial COLSON_PERFORMA = new WheelMaterial(
        "Colson Performa (85A)", 85, 1260, 0.70, 0.04, 0.60
    );
    
    /**
     * Fairlane (polyurethane) wheel (95A).
     */
    public static final WheelMaterial FAIRLANE_PU = new WheelMaterial(
        "Fairlane PU (95A)", 95, 1300, 0.60, 0.02, 0.50
    );
    
    /**
     * All predefined materials.
     */
    public static final WheelMaterial[] ALL_MATERIALS = {
        SOFT_COMPLIANT, MEDIUM_SOFT, STANDARD, STEALTH_BLUE,
        FIRM_GRIP, HARD, VERY_HARD, GREEN_COMPLIANT, COLSON_PERFORMA, FAIRLANE_PU
    };
    
    /**
     * Materials commonly used in FRC shooter flywheels.
     */
    public static final WheelMaterial[] SHOOTER_MATERIALS = {
        GREEN_COMPLIANT, SOFT_COMPLIANT, MEDIUM_SOFT, STANDARD, STEALTH_BLUE
    };
    
    /**
     * Creates a custom material with specified durometer.
     * Interpolates properties based on hardness.
     */
    public static WheelMaterial custom(String name, int durometer) {
        double t = (durometer - 30.0) / 65.0;
        t = Math.max(0, Math.min(1, t));
        
        double density = 1100 + 200 * t;
        double friction = 1.2 - 0.6 * t;
        double energyLoss = 0.20 - 0.18 * t;
        double grip = 0.95 - 0.45 * t;
        
        return new WheelMaterial(name, durometer, density, friction, energyLoss, grip);
    }
    
    /**
     * Calculates the effective coefficient of restitution for ball-wheel interaction.
     * Combines wheel properties with ball properties.
     */
    public double getEffectiveCOR(double ballCOR) {
        double wheelCOR = 1.0 - energyLossFactor;
        return Math.sqrt(ballCOR * wheelCOR);
    }
    
    /**
     * Estimates slip ratio based on compression and velocity.
     * Returns 0 for pure grip, 1 for pure slip.
     */
    public double estimateSlipRatio(double compressionRatio, double surfaceVelocityRatio) {
        double compressionGrip = Math.min(1.0, compressionRatio * 2.0);
        double velocitySlip = Math.max(0, surfaceVelocityRatio - 0.8) * 5.0;        
        double slipTendency = velocitySlip * (1.0 - gripFactor * compressionGrip);
        
        return Math.max(0, Math.min(1, slipTendency));
    }
    
    /**
     * Calculates energy transfer efficiency including slip.
     */
    public double getEnergyTransferEfficiency(double compressionRatio, double ballCOR) {
        double baseCOR = getEffectiveCOR(ballCOR);
        double slip = estimateSlipRatio(compressionRatio, 1.0);
        
        double deformationEfficiency = baseCOR * baseCOR;
        double slipEfficiency = 1.0 - slip * 0.3; // Change 0.3 ltr
        
        return deformationEfficiency * slipEfficiency * gripFactor;
    }
    
    // Getters
    public String getName() { return name; }
    public int getDurometer() { return durometer; }
    public double getDensity() { return density; }
    public double getFrictionCoefficient() { return frictionCoeff; }
    public double getEnergyLossFactor() { return energyLossFactor; }
    public double getGripFactor() { return gripFactor; }
    
    @Override
    public String toString() {
        return String.format("%s (%.0fA)", name, (double)durometer);
    }
}
