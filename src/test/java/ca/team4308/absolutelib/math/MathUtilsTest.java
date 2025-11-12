package ca.team4308.absolutelib.math;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Basic math utility tests
 */
public class MathUtilsTest {

    private static final double DELTA = 1e-6;

    @Test
    public void testAngleConversion() {
        // Test degrees to radians
        assertEquals(Math.PI, Math.toRadians(180.0), DELTA);
        assertEquals(Math.PI / 2, Math.toRadians(90.0), DELTA);
        assertEquals(0.0, Math.toRadians(0.0), DELTA);
        
        // Test radians to degrees
        assertEquals(180.0, Math.toDegrees(Math.PI), DELTA);
        assertEquals(90.0, Math.toDegrees(Math.PI / 2), DELTA);
        assertEquals(0.0, Math.toDegrees(0.0), DELTA);
    }

    @Test
    public void testRotationsToMeters() {
        double sprocketRadius = 0.025; // 2.5cm
        double gearRatio = 10.0;
        
        double rotations = 5.0;
        double meters = (rotations / gearRatio) * (2.0 * Math.PI * sprocketRadius);
        
        assertEquals(0.0785, meters, 0.001);
    }

    @Test
    public void testClamp() {
        // edu.wpi.first.math.MathUtil.clamp
        assertEquals(5.0, clamp(5.0, 0.0, 10.0), DELTA);
        assertEquals(0.0, clamp(-5.0, 0.0, 10.0), DELTA);
        assertEquals(10.0, clamp(15.0, 0.0, 10.0), DELTA);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Test
    public void testAngleWrapping() {
        // Test wrapping angles to [-180, 180]
        assertEquals(180.0, wrapAngle(180.0), DELTA);
        assertEquals(-180.0, wrapAngle(-180.0), DELTA);
        assertEquals(0.0, wrapAngle(360.0), DELTA);
        assertEquals(90.0, wrapAngle(450.0), DELTA);
        assertEquals(-90.0, wrapAngle(-450.0), DELTA);
    }

    private double wrapAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
