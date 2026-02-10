package ca.team4308.absolutelib.subsystems.simulation;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Base class for subsystem simulations with automatic logging.
 * All outputs are logged under the subsystem's path: /subsystems/(name)/simulation/...
 */
public abstract class SimulationBase extends AbsoluteSubsystem {

    /**
     * Simulation state container for a single mechanism
     */
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

    private static final String SIM_PREFIX = "simulation/";
    private double lastSimTimeSeconds = 0.0;
    private static final double DEFAULT_DT = 0.02;

    public enum LogLevel {
        LOW,
        MEDIUM,
        HIGH
    }

    private LogLevel logLevel = LogLevel.HIGH;

    public void setLogLevel(LogLevel level) {
        this.logLevel = level;
    }

    /**
     * Log simulation data with level filtering.
     */
    protected void logSim(LogLevel level, String key, Object value) {
        if (level.ordinal() <= logLevel.ordinal()) {
            recordOutput(SIM_PREFIX + key, value);
        }
    }

    /**
     * Record simulation output. Automatically prefixes with "simulation/".
     */
    protected void recordSimOutput(String key, Object value) {
        recordOutput(SIM_PREFIX + key, value);
    }

    public SimulationBase(String name) {
        setName(name);
    }

    public SimulationBase() {
        this("Simulation");
    }

    @Override
    protected void onInitialize() {
        lastSimTimeSeconds = getCurrentTimeSeconds();
        onSimulationInit();
        logInfo("Simulation initialized");
    }

    @Override
    public void periodic() {
        onPrePeriodic();

        double currentTime = getCurrentTimeSeconds();
        double dt = currentTime - lastSimTimeSeconds;
        if (dt <= 0.0) {
            dt = DEFAULT_DT; // fallback

        }
        lastSimTimeSeconds = currentTime;

        updateSimulation(dt);

        logSimulationState();

        onSimulationPeriodic(dt);

        onPostPeriodic();
    }

    /**
     * Update the simulation with a specific delta time.
     * Use this if driving the simulation manually from a subsystem.
     */
    public void update(double dtSeconds) {
        updateSimulation(dtSeconds);
        logSimulationState();
        onSimulationPeriodic(dtSeconds);
    }

    /**
     * Override to provide simulation state for logging
     */
    protected abstract SimState getSimulationState();

    /**
     * Override to update physics simulation each cycle
     */
    protected abstract void updateSimulation(double dtSeconds);

    /**
     * Hook: called once during simulation initialization
     */
    protected void onSimulationInit() {
    }

    /**
     * Hook: called every periodic cycle with delta time
     */
    protected void onSimulationPeriodic(double dtSeconds) {
    }

    /**
     * Logs all simulation data
     */
    private void logSimulationState() {
        SimState state = getSimulationState();
        if (state == null) {
            return;
        }

        // Core mechanical state (LOW)
        logSim(LogLevel.LOW, "positionMeters", state.positionMeters);

        // Standard state (MEDIUM)
        logSim(LogLevel.MEDIUM, "velocityMPS", state.velocityMetersPerSec);
        logSim(LogLevel.MEDIUM, "accelerationMPSS", state.accelerationMetersPerSecSq);

        // Electrical state (MEDIUM/HIGH)
        logSim(LogLevel.MEDIUM, "voltage", state.appliedVoltage);
        logSim(LogLevel.MEDIUM, "currentAmps", state.currentDrawAmps);
        logSim(LogLevel.HIGH, "temperatureC", state.temperatureCelsius);

        // Power consumption (HIGH)
        double powerWatts = state.appliedVoltage * state.currentDrawAmps;
        logSim(LogLevel.HIGH, "powerWatts", powerWatts);

        // Custom data (HIGH)
        if (state.customData != null && state.customDataKeys != null) {
            int n = Math.min(state.customData.length, state.customDataKeys.length);
            for (int i = 0; i < n; i++) {
                logSim(LogLevel.HIGH, "custom/" + state.customDataKeys[i], state.customData[i]);
            }
        }

        // Battery effects (optional)
        if (shouldSimulateBatteryEffects()) {
            simulateBatteryDraw(state.currentDrawAmps);
        }
    }

    /**
     * Helper: apply input voltage to simulation (override for custom behavior)
     */
    protected void applyInputVoltage(double volts) {
        // Subclasses implement motor/mechanism response
    }

    /**
     * Helper: set simulation position directly (for testing/init)
     */
    protected void setSimulationPosition(double meters) {
        // Subclasses implement
    }

    /**
     * Helper: set simulation velocity directly (for testing/init)
     */
    protected void setSimulationVelocity(double metersPerSec) {
        // Subclasses implement
    }

    /**
     * Override to enable battery voltage simulation (default: false in sim)
     */
    protected boolean shouldSimulateBatteryEffects() {
        return false;
    }

    /**
     * Simulate battery drain from current draw
     */
    private void simulateBatteryDraw(double currentAmps) {
        try {
            double newVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(currentAmps);
            RoboRioSim.setVInVoltage(newVoltage);
            recordSimOutput("batteryVoltage", newVoltage);
        } catch (Exception e) {
            // Graceful fallback if battery sim not available
        }
    }

    /**
     * Get current simulation time in seconds
     */
    protected double getCurrentTimeSeconds() {
        return RobotController.getFPGATime() / 1_000_000.0;
    }

    /**
     * Helper: clamp value to range
     */
    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Log a simulation event (throttled to avoid spam)
     */
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
    /**
     * Create a simple 1-DOF state for rotational mechanisms
     */
    protected SimState createRotationalState(double angleRad, double velocityRadPerSec,
            double voltage, double current) {
        SimState state = new SimState();
        state.positionMeters = angleRad; // treat as radians
        state.velocityMetersPerSec = velocityRadPerSec;
        state.appliedVoltage = voltage;
        state.currentDrawAmps = current;
        return state;
    }

    /**
     * Create a simple 1-DOF state for linear mechanisms
     */
    protected SimState createLinearState(double positionM, double velocityMPS,
            double voltage, double current) {
        SimState state = new SimState();
        state.positionMeters = positionM;
        state.velocityMetersPerSec = velocityMPS;
        state.appliedVoltage = voltage;
        state.currentDrawAmps = current;
        return state;
    }

    /**
     * Add custom telemetry to simulation state.
     *
     * @param state  the simulation state to modify
     * @param keys   telemetry key names
     * @param values telemetry values
     */
    protected void addCustomData(SimState state, String[] keys, double[] values) {
        state.customDataKeys = keys;
        state.customData = values;
    }

    /**
     * Log a 2D pose for visualization (e.g., arm end effector).
     *
     * @param key         telemetry key prefix
     * @param x           X position in meters
     * @param y           Y position in meters
     * @param rotationRad rotation in radians
     */
    protected void logPose2d(String key, double x, double y, double rotationRad) {
        recordSimOutput(key + "/x", x);
        recordSimOutput(key + "/y", y);
        recordSimOutput(key + "/rotationRad", rotationRad);
    }

    /**
     * Log multiple joint angles for multi-DOF mechanisms.
     *
     * @param anglesRad joint angles in radians
     */
    protected void logJointAngles(double... anglesRad) {
        recordSimOutput("jointAngles", anglesRad);
    }
}
