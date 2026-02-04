package frc.robot.subsystems;

import ca.team4308.absolutelib.subsystems.EndEffector;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Example end effector subsystem demonstrating intake/outtake control. Uses the
 * EndEffector base class for simple motor control.
 */
public class ExampleEndEffector extends AbsoluteSubsystem {

    private final EndEffector endEffector;
    private double currentSpeed = 0.0;

    // Speed constants
    private static final double INTAKE_SPEED = 0.8;
    private static final double OUTTAKE_SPEED = -0.6;
    private static final double HOLD_SPEED = 0.1;

    public ExampleEndEffector() {
        super();

        // Configure the end effector with a single motor
        MotorWrapper intakeMotor = new MotorWrapper(MotorType.TALONFX, 50);

        EndEffector.Config config = new EndEffector.Config()
                .withLeader(intakeMotor)
                .inverted(false);

        this.endEffector = new EndEffector(config);
        this.endEffector.initialize();
    }

    @Override
    public void periodic() {
        endEffector.periodic();
    }

    /**
     * Run the intake to pick up game pieces.
     */
    public Command intake() {
        return runOnce(() -> {
            endEffector.start(INTAKE_SPEED);
            currentSpeed = INTAKE_SPEED;
        });
    }

    /**
     * Run the outtake to eject game pieces.
     */
    public Command outtake() {
        return runOnce(() -> {
            endEffector.start(OUTTAKE_SPEED);
            currentSpeed = OUTTAKE_SPEED;
        });
    }

    /**
     * Hold game piece with low power.
     */
    public Command hold() {
        return runOnce(() -> {
            endEffector.start(HOLD_SPEED);
            currentSpeed = HOLD_SPEED;
        });
    }

    /**
     * Stop the end effector.
     */
    public Command stopCommand() {
        return runOnce(() -> {
            endEffector.stop();
            currentSpeed = 0.0;
        });
    }

    /**
     * Run intake while button is held, stop when released.
     */
    public Command intakeWhileHeld() {
        return run(() -> {
            endEffector.start(INTAKE_SPEED);
            currentSpeed = INTAKE_SPEED;
        }).finallyDo(() -> {
            endEffector.stop();
            currentSpeed = 0.0;
        });
    }

    public double getCurrentSpeed() {
        return currentSpeed;
    }

    @Override
    public Sendable log() {
        return builder -> {
            builder.setSmartDashboardType("ExampleEndEffector");
            builder.addDoubleProperty("CurrentSpeed", () -> currentSpeed, null);
            builder.addBooleanProperty("IsRunning", () -> currentSpeed != 0.0, null);
        };
    }
}
