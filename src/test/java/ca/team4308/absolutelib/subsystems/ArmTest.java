package ca.team4308.absolutelib.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorConfig;

@Disabled("Requires HAL")
@ExtendWith(MockitoExtension.class)
public class ArmTest {

    @Mock
    MotorWrapper motor;
    @Mock
    EncoderWrapper encoder;
    @Mock
    MotorConfig motorConfig;

    Arm arm;
    Arm.Joint joint;

    @BeforeEach
    public void setup() {
        arm = new Arm();
        Arm.JointConfig jointConfig = Arm.JointConfig.builder()
                .minAngleRad(-Math.PI)
                .maxAngleRad(Math.PI)
                .metersToRadians(1.0)
                .build();

        when(encoder.getPositionMeters()).thenReturn(0.0);
        joint = arm.addJoint(motor, motorConfig, encoder, jointConfig);
    }

    @Test
    public void testInitialState() {
        assertEquals(Arm.Joint.Mode.HOLDING, joint.getMode());
        assertEquals(0.0, joint.getAngleRadians(), 0.001);
    }

    @Test
    public void testSetTargetAngle() {
        joint.setTargetAngleRadians(1.0);
        assertEquals(Arm.Joint.Mode.POSITION, joint.getMode());
        assertEquals(1.0, joint.getTargetAngleRadians(), 0.001);
    }

    @Test
    public void testPeriodic() {
        joint.setTargetAngleRadians(1.0);
        when(encoder.getPositionMeters()).thenReturn(0.5); // Current pos

        arm.periodic();

        verify(motor).set(anyDouble());
    }

    @Test
    public void testStop() {
        joint.setTargetAngleRadians(1.0);
        arm.stop();
        assertEquals(Arm.Joint.Mode.IDLE, joint.getMode());
        verify(motor).set(0.0);
    }
}
