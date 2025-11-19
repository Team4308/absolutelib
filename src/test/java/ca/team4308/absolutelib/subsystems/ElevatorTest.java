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
public class ElevatorTest {

    @Mock
    MotorWrapper leader;
    @Mock
    MotorWrapper follower;
    @Mock
    EncoderWrapper encoder;
    @Mock
    MotorConfig motorConfig;

    Elevator elevator;

    @BeforeEach
    public void setup() {
        Elevator.ElevatorConfig config = Elevator.ElevatorConfig.builder()
                .minHeightMeters(0.0)
                .maxHeightMeters(1.0)
                .toleranceMeters(0.01)
                .build();

        when(encoder.getPositionMeters()).thenReturn(0.0);

        elevator = new Elevator(leader, motorConfig, encoder, config, follower);
    }

    @Test
    public void testInitialState() {
        assertEquals(Elevator.Mode.HOLDING, elevator.getMode());
        assertEquals(0.0, elevator.getCurrentPosition(), 0.001);
    }

    @Test
    public void testSetPosition() {
        elevator.setPosition(0.5);
        assertEquals(Elevator.Mode.POSITION, elevator.getMode());
        assertEquals(0.5, elevator.getTargetPosition(), 0.001);
    }

    @Test
    public void testSetManualPercent() {
        elevator.setManualPercent(0.5);
        assertEquals(Elevator.Mode.MANUAL, elevator.getMode());

        elevator.periodic();
        verify(leader).set(0.5);
    }

    @Test
    public void testStop() {
        elevator.setPosition(0.5);
        elevator.stop();
        assertEquals(Elevator.Mode.IDLE, elevator.getMode());
        verify(leader).set(0.0);
    }

    @Test
    public void testAtTarget() {
        elevator.setPosition(0.5);
        when(encoder.getPositionMeters()).thenReturn(0.5);
        assertTrue(elevator.atTarget());

        // Should transition to HOLDING if periodic is called while at target
        elevator.periodic();
        assertEquals(Elevator.Mode.HOLDING, elevator.getMode());
    }
}
