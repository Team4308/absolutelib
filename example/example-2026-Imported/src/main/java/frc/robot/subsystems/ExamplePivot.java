package frc.robot.subsystems;

import ca.team4308.absolutelib.subsystems.Pivot;
import ca.team4308.absolutelib.subsystems.simulation.PivotSimulation;
import ca.team4308.absolutelib.subsystems.simulation.SimulationBase.LogLevel;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExamplePivot extends AbsoluteSubsystem {

    private final Pivot pivot;

    public ExamplePivot() {
        super();
        MotorWrapper leader = new MotorWrapper(MotorType.TALONFX, 20);
        Pivot.Config pivotConfig = new Pivot.Config()
                .withLeader(leader)
                .withEncoder(EncoderWrapper.canCoder(21, 1.0)) // CANCoder ID 21
                .gear(50.0)
                .limits(-90, 90)
                .pid(55.0, 0.0, 0.1)
                .ff(0.5, 0.1, 0.1, 0.1)
                .useSmartMotion(false) // Smart motion is either MaxMotion or Magic Motion.
                .enableSimulation(true)
                .withSimulation(
                        new PivotSimulation.Config()
                                .gearbox(DCMotor.getKrakenX60(1), 1)
                                .gearRatio(50.0)
                                .armLength(0.5)
                                .armMass(3.0)
                                .limits(Math.toRadians(-90), Math.toRadians(90))
                                .startAngle(0.0)
                );

        this.pivot = new Pivot(pivotConfig);
        this.pivot.initialize(); // Ensures simulation setup runs
    }

    @Override
    public void periodic() {
        pivot.periodic();
    }

    public Command setAngle(double degrees) {
        return pivot.setPosition(degrees);
    }

    @Override
    public Sendable log() {
        return builder -> {
            builder.setSmartDashboardType("ExamplePivot");
            builder.addDoubleProperty("AngleDegrees", pivot::getAngleDeg, null);
            builder.addDoubleProperty("TargetAngleDegrees", pivot::getAngleDeg, null);
            builder.addBooleanProperty("AtTarget", pivot::atTarget, null);
        };
    }
}
