package ca.team4308.absolutelib.subsystems.simulation;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Base class for subsystem simulations with automatic AdvantageKit logging.
 * Extend this to create physics-based simulations that report all state.
 */
public abstract class SimulationBase extends AbsoluteSubsystem {

    /** Simulation state container for a single mechanism */
    public static class SimState {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double accelerationMetersPerSecSq = 0.0;
        public double appliedVoltage = 0.0;
        public double currentDrawAmps = 0.0;
        public double temperatureCelsius = 20.0;
        public double[] customData = new double[0];
        public String[] customDataKeys = new String[0];
    }

    private final String simLogPrefix;
    private double lastSimTimeSeconds = 0.0;
    private static final double DEFAULT_DT = 0.02; // 20ms default

    public SimulationBase(String name) {
        this.simLogPrefix = "/simulation/" + name;
    }

    public SimulationBase() {
        this("default");
    }

    @Override
    protected void onInitialize() {
        lastSimTimeSeconds = getCurrentTimeSeconds();
        onSimulationInit();
        logInfo("Simulation initialized: " + simLogPrefix);
    }

    @Override
    public void periodic() {
        onPrePeriodic();
        
        double currentTime = getCurrentTimeSeconds();
        double dt = currentTime - lastSimTimeSeconds;
        if (dt <= 0.0) dt = DEFAULT_DT; // fallback
        lastSimTimeSeconds = currentTime;

        updateSimulation(dt);
        
        logSimulationState();
        
        onSimulationPeriodic(dt);
        
        onPostPeriodic();
    }

    /** Override to provide simulation state for logging */
    protected abstract SimState getSimulationState();

    /** Override to update physics simulation each cycle */
    protected abstract void updateSimulation(double dtSeconds);

    /** Hook: called once during simulation initialization */
    protected void onSimulationInit() {}

    /** Hook: called every periodic cycle with delta time */
    protected void onSimulationPeriodic(double dtSeconds) {}

    /** Logs all simulation data to AdvantageKit */
    private void logSimulationState() {
        SimState state = getSimulationState();
        if (state == null) return;

        String prefix = simLogPrefix;
        
        // Core mechanical state
        recordOutput(prefix + "/positionMeters", state.positionMeters);
        recordOutput(prefix + "/velocityMPS", state.velocityMetersPerSec);
        recordOutput(prefix + "/accelerationMPSS", state.accelerationMetersPerSecSq);
        
        // Electrical state
        recordOutput(prefix + "/voltage", state.appliedVoltage);
        recordOutput(prefix + "/currentAmps", state.currentDrawAmps);
        recordOutput(prefix + "/temperatureC", state.temperatureCelsius);
        
        // Power consumption
        double powerWatts = state.appliedVoltage * state.currentDrawAmps;
        recordOutput(prefix + "/powerWatts", powerWatts);
        
        // Custom data
        if (state.customData != null && state.customDataKeys != null) {
            int n = Math.min(state.customData.length, state.customDataKeys.length);
            for (int i = 0; i < n; i++) {
                recordOutput(prefix + "/custom/" + state.customDataKeys[i], state.customData[i]);
            }
        }
        
        // Battery effects (optional, can be overridden)
        if (shouldSimulateBatteryEffects()) {
            simulateBatteryDraw(state.currentDrawAmps);
        }
    }

    /** Helper: apply input voltage to simulation (override for custom behavior) */
    protected void applyInputVoltage(double volts) {
        // Subclasses implement motor/mechanism response
    }

    /** Helper: set simulation position directly (for testing/init) */
    protected void setSimulationPosition(double meters) {
        // Subclasses implement
    }

    /** Helper: set simulation velocity directly (for testing/init) */
    protected void setSimulationVelocity(double metersPerSec) {
        // Subclasses implement
    }

    /** Override to enable battery voltage simulation (default: false in sim) */
    protected boolean shouldSimulateBatteryEffects() {
        return false;
    }

    /** Simulate battery drain from current draw */
    private void simulateBatteryDraw(double currentAmps) {
        try {
            // Use WPILib's battery sim
            double batteryVoltage = RoboRioSim.getVInVoltage();
            double newVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(currentAmps);
            RoboRioSim.setVInVoltage(newVoltage);
            recordOutput(simLogPrefix + "/batteryVoltage", newVoltage);
        } catch (Exception e) {
            // Graceful fallback if battery sim not available
        }
    }

    /** Get current simulation time in seconds */
    protected double getCurrentTimeSeconds() {
        return RobotController.getFPGATime() / 1_000_000.0;
    }

    /** Helper: clamp value to range */
    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Log a simulation event (throttled to avoid spam) */
    protected void logSimEvent(String event) {
        logThrottle("sim_event_" + event, 1000, "Sim event: " + event);
    }

    @Override
    protected void onStop() {
        logInfo("Simulation stopped");
    }

    @Override
    public Sendable log() {
        // Could return a Mechanism2d in the future
        return null;
    }

    // Helpers for common simulation patterns

    /** Create a simple 1-DOF state for rotational mechanisms */
    protected SimState createRotationalState(double angleRad, double velocityRadPerSec, 
                                            double voltage, double current) {
        SimState state = new SimState();
        state.positionMeters = angleRad; // treat as radians
        state.velocityMetersPerSec = velocityRadPerSec;
        state.appliedVoltage = voltage;
        state.currentDrawAmps = current;
        return state;
    }

    /** Create a simple 1-DOF state for linear mechanisms */
    protected SimState createLinearState(double positionM, double velocityMPS,
                                        double voltage, double current) {
        SimState state = new SimState();
        state.positionMeters = positionM;
        state.velocityMetersPerSec = velocityMPS;
        state.appliedVoltage = voltage;
        state.currentDrawAmps = current;
        return state;
    }

    /** Add custom telemetry to simulation state */
    protected void addCustomData(SimState state, String[] keys, double[] values) {
        state.customDataKeys = keys;
        state.customData = values;
    }

    /** Log a 2D pose for visualization (e.g., arm end effector) */
    protected void logPose2d(String key, double x, double y, double rotationRad) {
        recordOutput(simLogPrefix + "/" + key + "/x", x);
        recordOutput(simLogPrefix + "/" + key + "/y", y);
        recordOutput(simLogPrefix + "/" + key + "/rotationRad", rotationRad);
    }

    /** Log multiple joint angles for multi-DOF mechanisms */
    protected void logJointAngles(double... anglesRad) {
        recordOutput(simLogPrefix + "/jointAngles", anglesRad);
    }
}
