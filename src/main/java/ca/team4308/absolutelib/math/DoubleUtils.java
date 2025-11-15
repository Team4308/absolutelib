package ca.team4308.absolutelib.math;

/**
 * Utility methods for working with doubles in typical robotics control scenarios.
 * <p>
 * This class provides common operations such as clamping values and mapping
 * between ranges. All methods are static and side-effect free.
 */
public class DoubleUtils {

    /**
     * Normalizes a value into the range [-1.0, 1.0].
     * <ul>
     *   <li>If {@code d > 1.0}, returns {@code 1.0}.</li>
     *   <li>If {@code d < -1.0}, returns {@code -1.0}.</li>
     *   <li>Otherwise, returns {@code d} unchanged.</li>
     * </ul>
     *
     * @param d the input value
     * @return the value clamped to [-1.0, 1.0]
     */
    public static double normalize(double d) {
        if (d > 1.0) {
            return 1.0;
        }
        if (d < -1.0) {
            return -1.0;
        } else {
            return d;
        }
    }

    /**
     * Clamps {@code d} to the closed interval [{@code min}, {@code max}].
     * <ul>
     *   <li>If {@code d > max}, returns {@code max}.</li>
     *   <li>If {@code d < min}, returns {@code min}.</li>
     *   <li>Otherwise, returns {@code d} unchanged.</li>
     * </ul>
     *
     * @param d   the input value
     * @param min the minimum allowed value (inclusive)
     * @param max the maximum allowed value (inclusive)
     * @return {@code d} clamped to [{@code min}, {@code max}]
     */
    public static double clamp(double d, double min, double max) {
        if (d > max) {
            return max;
        } else if (d < min) {
            return min;
        } else {
            return d;
        }
    }

    /**
     * Linearly maps a value from one range into another.
     * <p>
     * Given an input {@code d} in the range [{@code old_min}, {@code old_max}],
     * returns a value in the range [{@code new_min}, {@code new_max}] such that:
     *
     * <pre>
     * d = old_min  -> new_min
     * d = old_max  -> new_max
     * </pre>
     *
     * and the mapping is linear in between.
     * <p>
     * This method does not clamp {@code d} to the input range.
     *
     * @param d        the value to map (typically in [{@code old_min}, {@code old_max}])
     * @param old_min  lower bound of the input range
     * @param old_max  upper bound of the input range
     * @param new_min  lower bound of the output range
     * @param new_max  upper bound of the output range
     * @return mapped value in the output range (possibly outside if {@code d} is outside the input range)
     * @throws ArithmeticException if {@code old_max == old_min}
     */
    public static double mapRange(double d, double old_min, double old_max, double new_min, double new_max) {
        return (new_min + (new_max - new_min) * (d - old_min) / (old_max - old_min));
    }

    /**
     * Alternate form of {@link #mapRange(double, double, double, double, double)} using different parameter names.
     * <p>
     * Given an input {@code x} in the range [{@code in_min}, {@code in_max}],
     * returns a value in the range [{@code out_min}, {@code out_max}] using the standard linear mapping:
     *
     * <pre>
     * out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
     * </pre>
     *
     * @param x        the value to map
     * @param in_min   lower bound of the input range
     * @param in_max   upper bound of the input range
     * @param out_min  lower bound of the output range
     * @param out_max  upper bound of the output range
     * @return mapped value in the output range
     * @throws ArithmeticException if {@code in_max == in_min}
     */
    public static double mapRangeNew(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
