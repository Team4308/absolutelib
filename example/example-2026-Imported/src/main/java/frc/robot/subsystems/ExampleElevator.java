package frc.robot.subsystems;

import ca.team4308.absolutelib.subsystems.Elevator;
import ca.team4308.absolutelib.subsystems.simulation.ElevatorSimulation;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleElevator extends AbsoluteSubsystem {

    private final Elevator elevator;

    public ExampleElevator() {
        MotorWrapper leader = new MotorWrapper(MotorType.SPARKMAX, 10);
        MotorWrapper follower = new MotorWrapper(MotorType.SPARKMAX, 11);
        leader.addFollower(follower);

        EncoderWrapper encoder = EncoderWrapper.ofMechanismRotations(
                leader::getPosition,
                (val) -> {
                    if (leader.isSparkMax()) {
                        leader.asSparkMax().getEncoder().setPosition(val);

                    }
                },
                0.0
        );

        Elevator.Config config = new Elevator.Config()
                .withLeader(leader)
                .withEncoder(encoder)
                .gear(1.0)
                .drumRadius(0.05)
                .limits(0.0, 2.0)
                .motion(0.1, 2.0);
                
                
            
            

        // Simulation Config
        ElevatorSimulation.ElevatorSimulationConfig simConfig = new ElevatorSimulation.ElevatorSimulationConfig();
        simConfig.leader = DCMotor.getNEO(2); // 2 NEOs
        simConfig.gearing = 1.0;
        simConfig.carriageMassKg = 5.0;
        simConfig.drumRadiusMeters = 0.05;
        simConfig.minHeightMeters = 0.0;
        simConfig.maxHeightMeters = 2.0;
        simConfig.simulateGravity = true;

        config.withSimulation(simConfig);

        this.elevator = new Elevator(config);
        this.elevator.initialize();
    }

    @Override
    public void periodic() {
        elevator.periodic();
    }

    public Command moveToHeight(double meters) {
        return elevator.setPosition(meters);
    }

    public void stop() {
        runOnce(elevator::stop);
    }

    @Override
    public Sendable log() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
}
