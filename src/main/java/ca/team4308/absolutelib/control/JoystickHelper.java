package ca.team4308.absolutelib.control;

import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.math.DoubleUtils;

public class JoystickHelper {
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

    public static boolean isStickCentered(Vector2 stickInput, double deadzone) {
        if (Math.abs(stickInput.x) < deadzone && Math.abs(stickInput.y) < deadzone) {
            return true;
        } else {
            return false;
        }
    }

    public static Vector2 scaleStick(Vector2 stickInput, double scale) {
        Vector2 newStickInput = new Vector2(Math.signum(stickInput.x) * Math.abs(Math.pow(stickInput.x, scale)),
                Math.signum(stickInput.y) * Math.abs(Math.pow(stickInput.y, scale)));
        return newStickInput;
    }

    public static Vector2 alternateScaleStick(Vector2 stickInput, double scale) {
        double mag = stickInput.magnitude();
        if (mag == 0) {
            return new Vector2();
        } else {
            Vector2 norm = new Vector2(stickInput.x / mag, stickInput.y / mag);
            return new Vector2(norm.x * Math.pow(mag, scale), norm.y * Math.pow(mag, scale));
        }
    }

    public static Vector2 precisionScaleStick(Vector2 stickInput, double scale, double precision) {
        double newX = (precision * Math.pow(stickInput.x, scale)) + ((1 - precision) * stickInput.x);
        double newY = (precision * Math.pow(stickInput.y, scale)) + ((1 - precision) * stickInput.y);
        return new Vector2(newX, newY);
    }

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

    public static Vector2 clampStick(Vector2 stickInput) {
        return new Vector2(DoubleUtils.clamp(stickInput.x, -1, 1), DoubleUtils.clamp(stickInput.y, -1, 1));
    }
}
