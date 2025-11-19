package ca.team4308.absolutelib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class DoubleUtilsTest {

    @Test
    public void testNormalize() {
        assertEquals(1.0, DoubleUtils.normalize(1.5), 0.0001);
        assertEquals(-1.0, DoubleUtils.normalize(-1.5), 0.0001);
        assertEquals(0.5, DoubleUtils.normalize(0.5), 0.0001);
        assertEquals(-0.5, DoubleUtils.normalize(-0.5), 0.0001);
        assertEquals(1.0, DoubleUtils.normalize(1.0), 0.0001);
        assertEquals(-1.0, DoubleUtils.normalize(-1.0), 0.0001);
    }

    @Test
    public void testClamp() {
        assertEquals(10.0, DoubleUtils.clamp(15.0, 0.0, 10.0), 0.0001);
        assertEquals(0.0, DoubleUtils.clamp(-5.0, 0.0, 10.0), 0.0001);
        assertEquals(5.0, DoubleUtils.clamp(5.0, 0.0, 10.0), 0.0001);
        assertEquals(0.0, DoubleUtils.clamp(0.0, 0.0, 10.0), 0.0001);
        assertEquals(10.0, DoubleUtils.clamp(10.0, 0.0, 10.0), 0.0001);
    }

    @Test
    public void testMapRange() {
        assertEquals(5.0, DoubleUtils.mapRange(0.5, 0.0, 1.0, 0.0, 10.0), 0.0001);
        assertEquals(0.0, DoubleUtils.mapRange(0.0, 0.0, 1.0, 0.0, 10.0), 0.0001);
        assertEquals(10.0, DoubleUtils.mapRange(1.0, 0.0, 1.0, 0.0, 10.0), 0.0001);
        assertEquals(15.0, DoubleUtils.mapRange(1.5, 0.0, 1.0, 0.0, 10.0), 0.0001); // Extrapolation
        assertEquals(-5.0, DoubleUtils.mapRange(-0.5, 0.0, 1.0, 0.0, 10.0), 0.0001); // Extrapolation
    }

    @Test
    public void testMapRangeNew() {
        assertEquals(5.0, DoubleUtils.mapRangeNew(0.5, 0.0, 1.0, 0.0, 10.0), 0.0001);
        assertEquals(0.0, DoubleUtils.mapRangeNew(0.0, 0.0, 1.0, 0.0, 10.0), 0.0001);
        assertEquals(10.0, DoubleUtils.mapRangeNew(1.0, 0.0, 1.0, 0.0, 10.0), 0.0001);
    }
}
