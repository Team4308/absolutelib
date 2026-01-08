package frc.robot.subsystems;

import ca.team4308.absolutelib.subsystems.Arm;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleArm extends AbsoluteSubsystem {

    private final Arm arm;
    
    private final Arm.Joint shoulder;
    private final Arm.Joint elbow;

    public ExampleArm() {
        super();
        this.arm = new Arm();
        MotorWrapper shoulderMotor = new MotorWrapper(MotorType.SPARKMAX, 30);
        // Use drumDiameter = 1/PI so getPositionMeters() returns rotations
        EncoderWrapper shoulderEncoder = EncoderWrapper.ofMechanismRotations(
                shoulderMotor::getPosition,
                (val) -> {
                    if (shoulderMotor.isSparkMax()) {
                        shoulderMotor.asSparkMax().getEncoder().setPosition(val);
                
                    }},
                1.0 / Math.PI
        );

        Arm.JointConfig shoulderConfig = Arm.JointConfig.builder()
                .minAngleRad(-Math.PI / 2)
                .maxAngleRad(Math.PI / 2)
                .metersToRadians(2 * Math.PI) // Rotations -> Radians
                .linkLengthMeters(1.0)
                .build();
        
        
        this.shoulder = arm.addJoint(shoulderMotor, null, shoulderEncoder, shoulderConfig);
        
        // PID And FF Tuning
        this.shoulder.setPositionPID(32, 0, 0);
        this.shoulder.setHoldPID(32, 0, 0);
        this.shoulder.setFeedforwardGains(0.1, 0.1, 0.1, 0.1);

        MotorWrapper elbowMotor = new MotorWrapper(MotorType.SPARKMAX, 32);
        EncoderWrapper elbowEncoder = EncoderWrapper.ofMechanismRotations(
                elbowMotor::getPosition,
                (val) -> {
                    if (elbowMotor.isSparkMax()) {
                        elbowMotor.asSparkMax().getEncoder().setPosition(val);
                
                    }},
                1.0 / Math.PI
        );

        Arm.JointConfig elbowConfig = Arm.JointConfig.builder()
                .minAngleRad(0.0)
                .maxAngleRad(Math.PI)
                .metersToRadians(2 * Math.PI)
                .linkLengthMeters(0.8)
                .build();
                

        this.elbow = arm.addJoint(elbowMotor, null, elbowEncoder, elbowConfig);

        // PID And FF Tuning
        this.elbow.setPositionPID(32, 0, 0);
        this.elbow.setHoldPID(32, 0, 0);
        this.elbow.setFeedforwardGains(0.1, 0.1, 0.1, 0.1);

        arm.enableSimulation(true);
        

        arm.initialize();
    }

    @Override
    public void periodic() {
        arm.periodic();
    }

    /**
     * Move to a specific (x, y) coordinate using Inverse Kinematics
     */
    public Command moveToPoint(double x, double y) {
        return runOnce(() -> arm.setGoalPose(x, y));
    }

    /**
     * Move joints manually to angles
     */
    public Command moveToAngles(double shoulderDeg, double elbowDeg) {
        return runOnce(() -> arm.setTargetAngles(Math.toRadians(shoulderDeg), Math.toRadians(elbowDeg)));
    }

    @Override
    public Sendable log() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
}
