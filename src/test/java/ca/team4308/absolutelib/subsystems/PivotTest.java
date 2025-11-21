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

@Disabled("Requires HAL")
@ExtendWith(MockitoExtension.class)
public class PivotTest {

    @Mock
    MotorWrapper leader;
    @Mock
    EncoderWrapper encoder;

    Pivot pivot;

    @BeforeEach
    public void setup() {
        Pivot.Config config = new Pivot.Config()
                .withLeader(leader)
                .withEncoder(encoder)
                .limits(-180, 180)
                .tolerance(1.0);

        pivot = new Pivot(config);
    }

    @Test
    public void testInitialState() {
        assertFalse(pivot.atTarget());
    }

    @Test
    public void testSetTargetAngle() {
        pivot.setTargetAngleDeg(90.0);

        when(encoder.getPositionMeters()).thenReturn(0.0); // 0 degrees (assuming 1:1 gear ratio default)
        pivot.periodic();

        verify(leader).setVoltage(anyDouble());
    }

    @Test
    public void testStop() {
        pivot.stop();
        verify(leader).setVoltage(0.0);
    }
}
