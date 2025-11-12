
package ca.team4308.absolutelib.subsystems.simulation;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class PivotSimulationTest {

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @Test
    void testSimulationInitialization() {
        PivotSimulation.Config config = new PivotSimulation.Config()
            .armLength(0.5)
            .armMass(3.0)
            .gearRatio(10.0);
        
        PivotSimulation sim = new PivotSimulation(config);
        sim.initialize();
        
        assertEquals(0.0, sim.getAngleDeg(), 0.1, "Initial angle should be 0");
    }

    @Test
    void testVoltageApplication() {
        PivotSimulation.Config config = new PivotSimulation.Config()
            .armLength(0.3)
            .armMass(2.0)
            .gravity(false); // Disable gravity for predictable test
        
        PivotSimulation sim = new PivotSimulation(config);
        sim.initialize();
        
        sim.setVoltage(6.0);
        
        // Run simulation
        for (int i = 0; i < 50; i++) {
            sim.periodic();
            SimHooks.stepTiming(0.02);
        }
        
        // Angle should have changed
        assertNotEquals(0.0, sim.getAngleDeg(), 0.5, "Angle should change with voltage");
    }

    @Test
    void testGravityEffect() {
        PivotSimulation.Config config = new PivotSimulation.Config()
            .armLength(0.5)
            .armMass(3.0)
            .gravity(true)
            .startAngle(Math.toRadians(90)); // Start horizontal
        
        PivotSimulation sim = new PivotSimulation(config);
        sim.initialize();
        
        double initialAngle = sim.getAngleDeg();
        
        sim.setVoltage(0.0);
        for (int i = 0; i < 50; i++) {
            sim.periodic();
            SimHooks.stepTiming(0.02);
        }
        
        assertTrue(sim.getAngleDeg() < initialAngle, "Arm should fall under gravity");
    }

    @Test
    void testAngleLimits() {
        PivotSimulation.Config config = new PivotSimulation.Config()
            .limits(Math.toRadians(-45), Math.toRadians(45))
            .gravity(false);
        
        PivotSimulation sim = new PivotSimulation(config);
        sim.initialize();
        
        sim.setVoltage(12.0);
        for (int i = 0; i < 200; i++) {
            sim.periodic();
            SimHooks.stepTiming(0.02);
        }
        
        assertTrue(sim.hasHitUpperLimit() || sim.getAngleDeg() <= 45.0,
            "Should respect upper limit");
    }

    @Test
    void testCurrentDraw() {
        PivotSimulation.Config config = new PivotSimulation.Config()
            .armLength(0.5)
            .armMass(5.0);
        
        PivotSimulation sim = new PivotSimulation(config);
        sim.initialize();
        
        sim.setVoltage(12.0);
        sim.periodic();
        
        double current = sim.getCurrentDrawAmps();
        assertTrue(current > 0, "Should draw current with voltage applied");
        assertTrue(current < 200, "Current should be reasonable");
    }

    @Test
    void testReset() {
        PivotSimulation.Config config = new PivotSimulation.Config()
            .startAngle(Math.toRadians(30));
        
        PivotSimulation sim = new PivotSimulation(config);
        sim.initialize();
        
        // Move the sim
        sim.setVoltage(6.0);
        for (int i = 0; i < 20; i++) {
            sim.periodic();
            SimHooks.stepTiming(0.02);
        }
        
        double movedAngle = sim.getAngleDeg();
        
        // Reset
        sim.reset();
        
        assertEquals(30.0, sim.getAngleDeg(), 0.1, "Should reset to start angle");
    }

    @Test
    void testPercentOutput() {
        PivotSimulation.Config config = new PivotSimulation.Config()
            .gravity(false);
        
        PivotSimulation sim = new PivotSimulation(config);
        sim.initialize();
        
        sim.setPercentOutput(0.5); // 50% = 6V
        sim.periodic();
        
        // Verify it's equivalent to 6V
        double angle1 = sim.getAngleDeg();
        
        sim.reset();
        sim.setVoltage(6.0);
        sim.periodic();
        
        assertEquals(angle1, sim.getAngleDeg(), 0.01, "Percent output should match voltage");
    }
}