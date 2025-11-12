package ca.team4308.absolutelib.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix.sensors.CANCoder;

import org.junit.jupiter.api.AfterEach;

import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class ArmTest {

    private Arm arm;
    private MotorWrapper motor1, motor2;
    private EncoderWrapper encoder1, encoder2;

    @BeforeEach
    void setup() {
        HAL.initialize(500, 0);
        
        motor1 = new MotorWrapper(MotorType.SPARKMAX,1);
        motor2 = new MotorWrapper(MotorType.SPARKMAX,1);
        encoder1 = EncoderWrapper.canCoder(1,1,4028,1); 
        encoder2 = EncoderWrapper.canCoder(1,1,4028,1); 
        
        arm = new Arm();
        arm.initialize();
    }

    @AfterEach
    void cleanup() {
        if (arm != null) {
            arm.stop();
        }
    }

    @Test
    void testAddJoint() {
        Arm.JointConfig config = Arm.JointConfig.builder()
            .linkLengthMeters(0.5)
            .minAngleRad(-Math.PI/2)
            .maxAngleRad(Math.PI/2)
            .build();
        
        Arm.Joint joint = arm.addJoint(motor1, null, encoder1, config);
        
        assertNotNull(joint);
        assertEquals(1, arm.getJoints().size());
    }

    @Test
    void testTwoJointArm() {
        Arm.JointConfig config1 = Arm.JointConfig.builder().linkLengthMeters(0.5).build();
        Arm.JointConfig config2 = Arm.JointConfig.builder().linkLengthMeters(0.4).build();
        
        arm.addJoint(motor1, null, encoder1, config1);
        arm.addJoint(motor2, null, encoder2, config2);
        
        assertEquals(2, arm.getJoints().size());
    }

    @Test
    void testSetTargetAngles() {
        Arm.JointConfig config = Arm.JointConfig.builder().build();
        arm.addJoint(motor1, null, encoder1, config);
        arm.addJoint(motor2, null, encoder2, config);
        
        assertDoesNotThrow(() -> {
            arm.setTargetAngles(Math.toRadians(30), Math.toRadians(45));
            arm.periodic();
        });
    }

    @Test
    void testSetManualPercents() {
        Arm.JointConfig config = Arm.JointConfig.builder().build();
        arm.addJoint(motor1, null, encoder1, config);
        arm.addJoint(motor2, null, encoder2, config);
        
        arm.setManualPercents(0.5, -0.3);
        arm.periodic();
        
        assertEquals(Arm.Joint.Mode.MANUAL, arm.getJoints().get(0).getMode());
        assertEquals(Arm.Joint.Mode.MANUAL, arm.getJoints().get(1).getMode());
    }

    @Test
    void testIKReachableGoal() {
        Arm.JointConfig config1 = Arm.JointConfig.builder()
            .linkLengthMeters(0.5)
            .minAngleRad(0)
            .maxAngleRad(Math.PI)
            .build();
        Arm.JointConfig config2 = Arm.JointConfig.builder()
            .linkLengthMeters(0.4)
            .minAngleRad(0)
            .maxAngleRad(Math.PI)
            .build();
        
        arm.addJoint(motor1, null, encoder1, config1);
        arm.addJoint(motor2, null, encoder2, config2);
        
        arm.setGoalPose(0.6, 0.3);
        
        assertTrue(arm.isIKMode());
        assertTrue(arm.isIKSolved());
    }

    @Test
    void testIKUnreachableGoal() {
        Arm.JointConfig config1 = Arm.JointConfig.builder().linkLengthMeters(0.5).build();
        Arm.JointConfig config2 = Arm.JointConfig.builder().linkLengthMeters(0.4).build();
        
        arm.addJoint(motor1, null, encoder1, config1);
        arm.addJoint(motor2, null, encoder2, config2);
        
        arm.setGoalPose(2.0, 2.0);
        
        assertTrue(arm.isIKMode());
        assertFalse(arm.isIKSolved());
    }

    @Test
    void testForwardKinematics() {
        Arm.JointConfig config1 = Arm.JointConfig.builder().linkLengthMeters(1.0).build();
        Arm.JointConfig config2 = Arm.JointConfig.builder().linkLengthMeters(1.0).build();
        
        arm.addJoint(motor1, null, encoder1, config1);
        arm.addJoint(motor2, null, encoder2, config2);
        
        arm.periodic();
        
        Translation2d endEffector = arm.getEndEffectorPose();
        
        assertEquals(2.0, endEffector.getX(), 0.1);
        assertEquals(0.0, endEffector.getY(), 0.1);
    }

    @Test
    void testJointModes() {
        Arm.JointConfig config = Arm.JointConfig.builder().build();
        Arm.Joint joint = arm.addJoint(motor1, null, encoder1, config);
        
        assertEquals(Arm.Joint.Mode.HOLDING, joint.getMode());
        
        joint.setManualPercent(0.5);
        assertEquals(Arm.Joint.Mode.MANUAL, joint.getMode());
        
        joint.setTargetAngleRadians(1.0);
        assertEquals(Arm.Joint.Mode.POSITION, joint.getMode());
        
        joint.stop();
        assertEquals(Arm.Joint.Mode.IDLE, joint.getMode());
    }

    @Test
    void testJointPIDConfiguration() {
        Arm.JointConfig config = Arm.JointConfig.builder().build();
        Arm.Joint joint = arm.addJoint(motor1, null, encoder1, config);
        
        assertDoesNotThrow(() -> {
            joint.setPositionPID(1.0, 0.0, 0.1);
            joint.setHoldPID(2.0, 0.0, 0.2);
            joint.setFeedforwardGains(0.1, 0.5, 0.01, 0.001);
        });
    }

    @Test
    void testJointOutputLimits() {
        Arm.JointConfig config = Arm.JointConfig.builder().build();
        Arm.Joint joint = arm.addJoint(motor1, null, encoder1, config);
        
        joint.setOutputLimits(-0.5, 0.5);
        joint.setManualPercent(1.0);
        
        assertDoesNotThrow(() -> arm.periodic());
    }

    @Test
    void testAllAtTargets() {
        Arm.JointConfig config = Arm.JointConfig.builder().toleranceRad(Math.toRadians(1.0)).build();
        
        arm.addJoint(motor1, null, encoder1, config);
        arm.addJoint(motor2, null, encoder2, config);
        

        arm.setTargetAngles(0, 0);
        
        for (int i = 0; i < 100; i++) {
            arm.periodic();
            SimHooks.stepTiming(0.02);
        }
        
        assertDoesNotThrow(() -> arm.allAtTargets());
    }

}
