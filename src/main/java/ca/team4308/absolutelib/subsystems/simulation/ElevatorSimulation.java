package ca.team4308.absolutelib.subsystems.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ElevatorSimulation extends SimulationBase {

    public static class ElevatorSimulationConfig {
        public DCMotor leader;
        public double gearing;
        public double carriageMassKg;
        public double minHeightMeters;
        public double maxHeightMeters;
        public double drumRadiusMeters;
        public boolean simulateGravity = true;
        public double startHeightMeters = 0.0;
        // optional tuning knob; not used directly by ElevatorSim
        public double dampingNmPerRadPerSec = 0.0;
    }

    private final ElevatorSim elevatorSim;
    private final Mechanism2d mech2d;
    private final MechanismRoot2d root;
    private final MechanismLigament2d elevatorLigament;
    private final ElevatorSimulationConfig config;
    private final SimState state = new SimState();

    private double appliedVoltage = 0.0;

    public ElevatorSimulation(ElevatorSimulationConfig config) {
        super("elevator");
        this.config = config;

        elevatorSim = new ElevatorSim(
            config.leader,
            config.gearing,
            config.carriageMassKg,
            config.drumRadiusMeters,
            config.minHeightMeters,
            config.maxHeightMeters,
            config.simulateGravity,
            config.startHeightMeters
        );

        mech2d = new Mechanism2d(2.0, config.maxHeightMeters + 0.5);
        root = mech2d.getRoot("ElevatorBase", 1.0, config.minHeightMeters);
        elevatorLigament = root.append(
            new MechanismLigament2d("Carriage", config.startHeightMeters, 90)
        );
    }

    /** Command voltage to the simulated elevator motor. */
    public void setInputVoltage(double volts) {
        appliedVoltage = volts;
    }

    /**
     * Public update method called by subsystems (e.g. Elevator.java).
     */
    public void simUpdate(double dtSeconds) {
        updateSimulation(dtSeconds);
        onSimulationPeriodic(dtSeconds);
    }

    @Override
    protected void onSimulationInit() {
        elevatorSim.setState(config.startHeightMeters, 0.0);
        logInfo("Elevator simulation initialized");
    }

    @Override
    protected void updateSimulation(double dtSeconds) {
        double clamped = clamp(appliedVoltage, -12.0, 12.0);
        elevatorSim.setInputVoltage(clamped);
        elevatorSim.update(dtSeconds);

        // Update state snapshot
        state.positionMeters = elevatorSim.getPositionMeters();
        state.velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        state.appliedVoltage = appliedVoltage;
        state.currentDrawAmps = elevatorSim.getCurrentDrawAmps();
        state.temperatureCelsius = 20.0 + state.currentDrawAmps * 1.5;

        // Update mech2d
        elevatorLigament.setLength(elevatorSim.getPositionMeters());

        recordOutput("positionMeters", state.positionMeters);
        recordOutput("velocityMetersPerSec", state.velocityMetersPerSec);
        recordOutput("currentAmps", state.currentDrawAmps);
        recordOutput("appliedVoltage", state.appliedVoltage);
        recordOutput("mechanism2d/object", mech2d);
    }

    @Override
    protected SimState getSimulationState() {
        return state;
    }

    public Mechanism2d getMechanism2d() {
        return mech2d;
    }

    public double getPositionMeters() {
        return elevatorSim.getPositionMeters();
    }

    public double getVelocityMetersPerSecond() {
        return elevatorSim.getVelocityMetersPerSecond();
    }
}
