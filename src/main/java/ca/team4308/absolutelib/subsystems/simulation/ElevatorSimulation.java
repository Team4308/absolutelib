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
        public double elevatorPulleyRadiusMeters;
        public boolean simulateGravity = true;
    }

    private final ElevatorSim elevatorSim;
    private final Mechanism2d mech2d;
    private final Mechanism2d elevatormech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d elevatorLigament;
    private final ElevatorSimulationConfig config;

    public ElevatorSimulation(ElevatorSimulationConfig config) {
        this.config = config;
        elevatorSim = new ElevatorSim(config.leader, config.gearing, config.carriageMassKg, config.drumRadiusMeters, config.minHeightMeters, config.maxHeightMeters, true, config.minHeightMeters, null);

        mech2d = new Mechanism2d(2.0, config.maxHeightMeters);
        elevatormech = new Mechanism2d(2.0, config.maxHeightMeters);
        root = mech2d.getRoot("ElevatorRoot", 1.0, config.minHeightMeters);
        elevatorLigament = root.append(new MechanismLigament2d("Elevator", config.minHeightMeters, 90));
    }

    public void setInputVoltage(double volts) {
        elevatorSim.setInputVoltage(volts);
    }

    public void update(double dtSeconds) {
        elevatorSim.update(dtSeconds);
        elevatorLigament.setLength(elevatorSim.getPositionMeters());
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

    @Override
    protected SimState getSimulationState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSimulationState'");
    }

    @Override
    protected void updateSimulation(double dtSeconds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateSimulation'");
    }

}
