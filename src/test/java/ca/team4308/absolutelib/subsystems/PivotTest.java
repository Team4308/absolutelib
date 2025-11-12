package ca.team4308.absolutelib.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterEach;

import ca.team4308.absolutelib.subsystems.simulation.PivotSimulation;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class PivotTest {

    private Pivot pivot;
    private MotorWrapper motor;
    private EncoderWrapper encoder;

    @BeforeEach
    void setup() {
        HAL.initialize(500, 0);
        
        motor = new MotorWrapper(MotorWrapper.MotorType.SPARKMAX, 1);
        encoder = new EncoderWrapper() {
            public double position = 0.0;
            @Override
            public double getPositionMeters() { return position; }
            @Override
            public void setPositionMeters(double meters) { position = meters; }
        };
        
        Pivot.Config config = new Pivot.Config()
            .withLeader(motor)
            .pid(1.0, 0.0, 0.1)
            .ff(0.0, 0.5, 0.0, 0.0)
            .gear(10.0)
            .limits(-90, 90)
            .tolerance(2.0)
            .enableSimulation(false);
        
        pivot = new Pivot(config);
        pivot.initialize();
    }

    @AfterEach
    void cleanup() {
        if (pivot != null) {
            pivot.stop();
        }
    }

    @Test
    void testInitialization() {
        assertNotNull(pivot);
        assertEquals(0.0, pivot.getAngleDeg(), 0.1);
    }

    @Test
    void testSetTargetAngle() {
        pivot.setTargetAngleDeg(45.0);
        
        for (int i = 0; i < 10; i++) {
            pivot.periodic();
            SimHooks.stepTiming(0.02);
        }
        
    }

    @Test
    void testAngleLimits() {
        assertDoesNotThrow(() -> {
            pivot.setTargetAngleDeg(100.0);
            pivot.periodic();
        });
        
        assertDoesNotThrow(() -> {
            pivot.setTargetAngleDeg(-100.0);
            pivot.periodic();
        });
    }

    @Test
    void testManualVoltageMode() {
        pivot.setManualVoltage(6.0);
        pivot.periodic();
        
    }

    @Test
    void testDisable() {
        pivot.setTargetAngleDeg(45.0);
        pivot.periodic();
        
        pivot.disable();
        pivot.periodic();
        
    }

    @Test
    void testZeroEncoder() {
        assertEquals(5.0, encoder.getPositionMeters(), 0.01);
        
        pivot.zeroEncoder();
        assertEquals(0.0, encoder.getPositionMeters(), 0.01);
    }

    @Test
    void testBrakeMode() {
        assertDoesNotThrow(() -> {
            pivot.setBrakeMode(true);
        
            pivot.setBrakeMode(false);
        });
    }

    @Test
    void testPIDUpdate() {
        assertDoesNotThrow(() -> pivot.updatePID(2.0, 0.1, 0.2));
    }

    @Test
    void testFFUpdate() {
        assertDoesNotThrow(() -> pivot.updateFF(0.1, 0.6, 0.05, 0.01));
    }

}
