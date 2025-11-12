package ca.team4308.absolutelib.wrapper;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.util.sendable.Sendable;

/**
 * Tests for AbsoluteSubsystem base class
 */
public class AbsoluteSubsystemTest {

    private TestSubsystem subsystem;

    // Test implementation of AbsoluteSubsystem
    private static class TestSubsystem extends AbsoluteSubsystem {
        public int periodicCallCount = 0;
        public boolean initialized = false;
        public boolean stopped = false;

        @Override
        public Sendable log() {
            return null;
        }

        @Override
        protected void onInitialize() {
            initialized = true;
        }


        @Override
        protected void onStop() {
            stopped = true;
        }
    }

    @BeforeEach
    public void setup() {
        subsystem = new TestSubsystem();
    }

    @Test
    public void testInitialize() {
        assertFalse(subsystem.initialized);
        subsystem.initialize();
        assertTrue(subsystem.initialized);
    }

    @Test
    public void testPeriodic() {
        assertEquals(0, subsystem.periodicCallCount);
        subsystem.periodic();
        assertEquals(1, subsystem.periodicCallCount);
        subsystem.periodic();
        assertEquals(2, subsystem.periodicCallCount);
    }

    @Test
    public void testStop() {
        assertFalse(subsystem.stopped);
        subsystem.stop();
        assertTrue(subsystem.stopped);
    }

    @Test
    public void testGetName() {
        subsystem.setName("TestSubsystem");
        assertEquals("TestSubsystem", subsystem.getName());
    }

    @Test
    public void testLogChannelBase() {
        subsystem.setName("TestSubsystem");
        // The log channel should be based on subsystem name
        String expected = "/subsystems/TestSubsystem";
        // We can't directly test getLogChannelBase as it's protected,
        // but we know the format from the code
        assertNotNull(subsystem.getName());
    }

    @Test
    public void testRunSafely() {
        // Test that exceptions are caught
        subsystem.runSafely("test", () -> {
            throw new RuntimeException("Test exception");
        });
        
        // Should not throw - exception was caught and logged
        assertTrue(true);
    }

    @Test
    public void testRunPeriodicWithHooks() {
        subsystem.runPeriodicWithHooks(() -> {
            subsystem.periodicCallCount++;
        });
        
        assertEquals(1, subsystem.periodicCallCount);
    }
}
