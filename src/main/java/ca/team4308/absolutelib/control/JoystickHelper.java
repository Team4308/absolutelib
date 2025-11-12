package ca.team4308.absolutelib.control;

import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.math.DoubleUtils;

/**
 * Utility class for processing joystick inputs with various deadzone and scaling algorithms.
 * Provides methods to handle different types of deadzones and input scaling for better control.
 */
public class JoystickHelper {

    /**
     * Applies a sloped scaled axial deadzone to the joystick input.
     * The deadzone varies based on the magnitude of the perpendicular axis.
     * 
     * @param stickInput The input vector from the joystick
     * @param deadzone The deadzone value to apply (0.0 to 1.0)
     * @return A new Vector2 with the processed input values
     */
    public static Vector2 SlopedScaledAxialDeadzone(Vector2 stickInput, double deadzone) {
        double deadzoneX = deadzone * Math.abs(stickInput.y);
        double deadzoneY = deadzone * Math.abs(stickInput.x);

        Vector2 result = new Vector2(0.0, 0.0);
        Vector2 sign = new Vector2(Math.signum(stickInput.x), Math.signum(stickInput.y));

        if (Math.abs(stickInput.x) > deadzoneX) {
            result.x = sign.x * DoubleUtils.mapRangeNew(Math.abs(stickInput.x), deadzoneX, 1, 0, 1);
        }
        if (Math.abs(stickInput.y) > deadzoneY) {
            result.y = sign.y * DoubleUtils.mapRangeNew(Math.abs(stickInput.y), deadzoneY, 1, 0, 1);
        }
        return result;
    }

    /**
     * Applies a scaled radial deadzone to the joystick input.
     * Creates a circular deadzone around the center of the stick.
     * 
     * @param stickInput The input vector from the joystick
     * @param deadzone The deadzone value to apply (0.0 to 1.0)
     * @return A new Vector2 with the processed input values
     */
    public static Vector2 ScaledRadialDeadzone(Vector2 stickInput, double deadzone) {
        double inputMagnitude = stickInput.magnitude();
        if (inputMagnitude < deadzone) {
            return new Vector2(0.0, 0.0);
        } else {
            double legalRange = 1.0 - deadzone;
            double normalizedMag = Math.min(1.0, (inputMagnitude - deadzone) / legalRange);
            double scale = normalizedMag / inputMagnitude;
            return new Vector2(stickInput.normalize().x * scale, stickInput.normalize().y * scale);
        }
    }

    /**
     * Combines radial and axial deadzones for a hybrid approach.
     * First applies a radial deadzone, then an axial deadzone to the result.
     * 
     * @param stickInput The input vector from the joystick
     * @param deadzone The deadzone value to apply (0.0 to 1.0)
     * @return A new Vector2 with the processed input values
     */
    public static Vector2 HybridDeadzone(Vector2 stickInput, double deadzone) {
        double inputMagnitude = stickInput.magnitude();
        if (inputMagnitude < deadzone) {
            return new Vector2(0.0, 0.0);
        } else {
            Vector2 partialOutput = ScaledRadialDeadzone(stickInput, deadzone);
            Vector2 finalOutput = SlopedScaledAxialDeadzone(partialOutput, deadzone);
            return finalOutput;
        }
    }

    /**
     * Applies a simple axial deadzone independently to each axis.
     * Values below the deadzone are set to zero.
     * 
     * @param stickInput The input vector from the joystick
     * @param deadzone The deadzone value to apply (0.0 to 1.0)
     * @return A new Vector2 with the processed input values
     */
    public static Vector2 AxialDeadzone(Vector2 stickInput, double deadzone) {
        Vector2 newStickInput = new Vector2(stickInput.x, stickInput.y);
        if (Math.abs(newStickInput.x) < deadzone) {
            newStickInput.x = 0.0;
        }
        if (Math.abs(newStickInput.y) < deadzone) {
            newStickInput.y = 0.0;
        }
        return newStickInput;
    }

    /**
     * Applies a scaled axial deadzone to each axis independently.
     * Rescales the output to maintain smooth transitions.
     * 
     * @param stickInput The input vector from the joystick
     * @param deadzone The deadzone value to apply (0.0 to 1.0)
     * @return A new Vector2 with the processed input values
     */
    public static Vector2 ScaledAxialDeadzone(Vector2 stickInput, double deadzone) {
        Vector2 result = new Vector2(0.0, 0.0);
        Vector2 sign = new Vector2(Math.signum(stickInput.x), Math.signum(stickInput.y));

        if (Math.abs(stickInput.x) > deadzone) {
            result.x = sign.x * DoubleUtils.mapRangeNew(Math.abs(stickInput.x), deadzone, 1, 0, 1);
        }
        if (Math.abs(stickInput.y) > deadzone) {
            result.y = sign.y * DoubleUtils.mapRangeNew(Math.abs(stickInput.y), deadzone, 1, 0, 1);
        }

        return result;
    }

    /**
     * Checks if the joystick input is within the deadzone (centered).
     * 
     * @param stickInput The input vector from the joystick
     * @param deadzone The deadzone value to check against (0.0 to 1.0)
     * @return true if the stick is within the deadzone, false otherwise
     */
    public static boolean isStickCentered(Vector2 stickInput, double deadzone) {
        if (Math.abs(stickInput.x) < deadzone && Math.abs(stickInput.y) < deadzone) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Applies exponential scaling to the joystick input.
     * 
     * @param stickInput The input vector from the joystick
     * @param scale The exponential scale factor
     * @return A new Vector2 with the scaled input values
     */
    public static Vector2 scaleStick(Vector2 stickInput, double scale) {
        Vector2 newStickInput = new Vector2(Math.signum(stickInput.x) * Math.abs(Math.pow(stickInput.x, scale)),
                Math.signum(stickInput.y) * Math.abs(Math.pow(stickInput.y, scale)));
        return newStickInput;
    }

    /**
     * Applies alternative exponential scaling that preserves the input direction.
     * Scales the magnitude while maintaining the angle of the input.
     * 
     * @param stickInput The input vector from the joystick
     * @param scale The exponential scale factor
     * @return A new Vector2 with the scaled input values
     */
    public static Vector2 alternateScaleStick(Vector2 stickInput, double scale) {
        double mag = stickInput.magnitude();
        if (mag == 0) {
            return new Vector2();
        } else {
            Vector2 norm = new Vector2(stickInput.x / mag, stickInput.y / mag);
            return new Vector2(norm.x * Math.pow(mag, scale), norm.y * Math.pow(mag, scale));
        }
    }

    /**
     * Applies precision-weighted exponential scaling to the joystick input.
     * Blends between linear and exponential response based on precision factor.
     * 
     * @param stickInput The input vector from the joystick
     * @param scale The exponential scale factor
     * @param precision The blend factor between linear (0.0) and exponential (1.0) scaling
     * @return A new Vector2 with the scaled input values
     */
    public static Vector2 precisionScaleStick(Vector2 stickInput, double scale, double precision) {
        double newX = (precision * Math.pow(stickInput.x, scale)) + ((1 - precision) * stickInput.x);
        double newY = (precision * Math.pow(stickInput.y, scale)) + ((1 - precision) * stickInput.y);
        return new Vector2(newX, newY);
    }

    /**
     * Applies a cubic function with deadzone and precision scaling.
     * Provides smooth transitions and precise control.
     * 
     * @param stickInput The input vector from the joystick
     * @param deadzone The deadzone value to apply (0.0 to 1.0)
     * @param precision The precision factor for control sensitivity
     * @param scale The exponential scale factor
     * @return A new Vector2 with the processed input values
     */
    public static Vector2 cubicDeadzone(Vector2 stickInput, double deadzone, double precision, double scale) {
        double x = stickInput.x;
        double y = stickInput.y;

        double d = deadzone;
        double w = precision;

        double newX = ((w * (Math.pow(x, scale)) + (1.0 - w) * x)
                - (Math.abs(x) / x) * (w * (Math.pow(d, scale)) + (1.0 - w) * d))
                / (1.0 - (w * (Math.pow(d, scale)) + (1.0 - w) * d));
        double newY = ((w * (Math.pow(y, scale)) + (1.0 - w) * y)
                - (Math.abs(y) / y) * (w * (Math.pow(d, scale)) + (1.0 - w) * d))
                / (1.0 - (w * (Math.pow(d, scale)) + (1.0 - w) * d));

        return new Vector2(newX, newY);
    }

    /**
     * Clamps the joystick input values to the range [-1, 1].
     * 
     * @param stickInput The input vector from the joystick
     * @return A new Vector2 with the clamped input values
     */
    public static Vector2 clampStick(Vector2 stickInput) {
        return new Vector2(DoubleUtils.clamp(stickInput.x, -1, 1), DoubleUtils.clamp(stickInput.y, -1, 1));
    }
}
