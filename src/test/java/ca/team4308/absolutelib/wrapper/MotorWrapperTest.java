package ca.team4308.absolutelib.wrapper;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for MotorWrapper - basic validation only since hardware is not available
 */
public class MotorWrapperTest {

    @Test
    public void testMotorTypeEnum() {
        // Test that all motor types are defined
        assertNotNull(MotorWrapper.MotorType.TALONFX);
        assertNotNull(MotorWrapper.MotorType.SPARKMAX);
        
        // Verify enum values
        assertEquals(2, MotorWrapper.MotorType.values().length);
    }

    @Test
    public void testSparkMaxTypeEnum() {
    }

    @Test
    public void testMotorTypeToString() {
        assertEquals("TALONFX", MotorWrapper.MotorType.TALONFX.toString());
    }
}
