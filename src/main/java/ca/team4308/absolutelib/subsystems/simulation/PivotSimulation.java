package ca.team4308.absolutelib.subsystems.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class PivotSimulation extends SimulationBase {

    /** Configuration for pivot simulation physics */
    public static class Config {
        public DCMotor gearbox = DCMotor.getNEO(1);
        public double gearRatio = 1.0;
        public double armLengthMeters = 0.5;
        public double armMassKg = 5.0;
        public double minAngleRad = -Math.PI;
        public double maxAngleRad = Math.PI;
        public boolean simulateGravity = true;
        public double startAngleRad = 0.0;

        public Config gearbox(DCMotor motor, int count) { this.gearbox = DCMotor.getNEO(count); return this; }
        public Config gearRatio(double ratio) { this.gearRatio = ratio; return this; }
        public Config armLength(double meters) { this.armLengthMeters = meters; return this; }
        public Config armMass(double kg) { this.armMassKg = kg; return this; }
        public Config limits(double minRad, double maxRad) { this.minAngleRad = minRad; this.maxAngleRad = maxRad; return this; }
        public Config startAngle(double rad) { this.startAngleRad = rad; return this; }
        public Config gravity(boolean enable) { this.simulateGravity = enable; return this; }

        // Helper factory method
        public static Config fromPivotConfig(
            ca.team4308.absolutelib.subsystems.Pivot.Config pivotCfg,
            DCMotor motorType,
            int motorCount,
            double armLengthM,
            double armMassKg
        ) {
            Config simCfg = new Config();
            simCfg.gearbox = motorType;
            simCfg.gearRatio = pivotCfg.gearRatio;
            simCfg.minAngleRad = Math.toRadians(pivotCfg.minAngleDeg);
            simCfg.maxAngleRad = Math.toRadians(pivotCfg.maxAngleDeg);
            simCfg.startAngleRad = Math.toRadians(pivotCfg.angleOffsetDeg);
            simCfg.armLengthMeters = armLengthM;
            simCfg.armMassKg = armMassKg;
            return simCfg;
        }
    }

    private final SingleJointedArmSim armSim;
    private final Config config;
    private double appliedVoltage = 0.0;
    private final SimState currentState = new SimState();

    public PivotSimulation(String name, Config config) {
        super(name);
        this.config = config;
        
        double moi = (1.0 / 3.0) * config.armMassKg * config.armLengthMeters * config.armLengthMeters;
        
        this.armSim = new SingleJointedArmSim(
            config.gearbox,
            config.gearRatio,
            moi,
            config.armLengthMeters,
            config.minAngleRad,
            config.maxAngleRad,
            config.simulateGravity,
            config.startAngleRad
        );
    }

    public PivotSimulation(Config config) {
        this("pivot", config);
    }

    @Override
    protected void onSimulationInit() {
        armSim.setState(config.startAngleRad, 0.0);
        logInfo("Pivot simulation started at " + Math.toDegrees(config.startAngleRad) + " degrees");
    }

    @Override
    protected void updateSimulation(double dtSeconds) {
        double clampedVoltage = clamp(appliedVoltage, -12.0, 12.0);
        armSim.setInputVoltage(clampedVoltage);
        
        armSim.update(dtSeconds);
        
        currentState.positionMeters = armSim.getAngleRads();
        currentState.velocityMetersPerSec = armSim.getVelocityRadPerSec();
        currentState.appliedVoltage = clampedVoltage;
        currentState.currentDrawAmps = armSim.getCurrentDrawAmps();
        currentState.accelerationMetersPerSecSq = 0.0; 
        currentState.temperatureCelsius = 20.0 + (currentState.currentDrawAmps * 2.0);
        
    }

    @Override
    protected SimState getSimulationState() {
        return currentState;
    }

    @Override
    protected void onSimulationPeriodic(double dtSeconds) {
        // Log additional pivot-specific data
        recordOutput("angleDeg", Math.toDegrees(armSim.getAngleRads()));
        recordOutput("velocityDegPerSec", Math.toDegrees(armSim.getVelocityRadPerSec()));
        recordOutput("hasHitLowerLimit", armSim.hasHitLowerLimit());
        recordOutput("hasHitUpperLimit", armSim.hasHitUpperLimit());
        
        // Log end effector position for visualization
        double angle = armSim.getAngleRads();
        double endX = config.armLengthMeters * Math.cos(angle);
        double endY = config.armLengthMeters * Math.sin(angle);
        logPose2d("endEffector", endX, endY, angle);
        
        // Log mechanism for AdvantageScope
        double[] ligament = new double[] { config.armLengthMeters, Math.toDegrees(angle) };
        recordOutput(getLogChannelBase() + "/mechanism2d", ligament);
    }

    // Public control methods

    /** Apply voltage to the simulated motor */
    @Override
    protected void applyInputVoltage(double volts) {
        this.appliedVoltage = volts;
    }

    /** Set voltage directly (alias for consistency) */
    public void setVoltage(double volts) {
        applyInputVoltage(volts);
    }

    /** Set the simulated angle directly (for testing/initialization) */
    @Override
    public void setSimulationPosition(double angleRad) {
        armSim.setState(angleRad, armSim.getVelocityRadPerSec());
    }

    /** Set the simulated angular velocity directly */
    @Override
    protected void setSimulationVelocity(double radPerSec) {
        armSim.setState(armSim.getAngleRads(), radPerSec);
    }

    // Getters for current state

    public double getAngleRad() {
        return armSim.getAngleRads();
    }

    public double getAngleDeg() {
        return Math.toDegrees(armSim.getAngleRads());
    }

    public double getVelocityRadPerSec() {
        return armSim.getVelocityRadPerSec();
    }

    public double getCurrentDrawAmps() {
        return armSim.getCurrentDrawAmps();
    }

    public boolean hasHitLowerLimit() {
        return armSim.hasHitLowerLimit();
    }

    public double getVoltage() {
        return appliedVoltage;
    }

    public boolean hasHitUpperLimit() {
        return armSim.hasHitUpperLimit();
    }

    /** Reset simulation to initial state */
    public void reset() {
        armSim.setState(config.startAngleRad, 0.0);
        appliedVoltage = 0.0;
        logInfo("Pivot simulation reset");
    }

    /** Apply a percent output (-1.0 to 1.0) converted to voltage */
    public void setPercentOutput(double percent) {
        setVoltage(clamp(percent, -1.0, 1.0) * 12.0);
    }

    @Override
    protected void onStop() {
        appliedVoltage = 0.0;
        super.onStop();
    }

    public static Config fromPivotConfig(ca.team4308.absolutelib.subsystems.Pivot.Config pivotCfg,
                                         DCMotor motor, int motorCount, double armLengthM, double armMassKg) {
        Config simCfg = new Config();
        simCfg.gearbox = DCMotor.getNEO(motorCount); // or pass motor type
        simCfg.gearRatio = pivotCfg.gearRatio;
        simCfg.minAngleRad = Math.toRadians(pivotCfg.minAngleDeg);
        simCfg.maxAngleRad = Math.toRadians(pivotCfg.maxAngleDeg);
        simCfg.startAngleRad = Math.toRadians(pivotCfg.angleOffsetDeg);
        simCfg.armLengthMeters = armLengthM;
        simCfg.armMassKg = armMassKg;
        return simCfg;
    }
}
