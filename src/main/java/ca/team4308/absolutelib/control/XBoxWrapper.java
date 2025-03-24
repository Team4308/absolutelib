package ca.team4308.absolutelib.control;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Wrapper class for Xbox controller functionality in FRC.
 * Provides easy access to buttons and analog inputs.
 */
public class XBoxWrapper {
    /**
     * Static mapping of Xbox controller button IDs.
     */
    public static class XBoxMapping {
        public static int A = 1;
        public static int B = 2;
        public static int X = 3;
        public static int Y = 4;

        public static int LB = 5;
        public static int RB = 6;

        public static int Start = 7;
        public static int Back = 8;

        public static int LeftStickButton = 9;
        public static int RightStickButton = 10;
    }

    public final Joystick joystick;

    public final JoystickButton A;
    public final JoystickButton B;
    public final JoystickButton X;
    public final JoystickButton Y;

    public final JoystickButton LB;
    public final JoystickButton RB;

    public final JoystickButton Start;
    public final JoystickButton Back;

    public final JoystickButton LeftStickButton;
    public final JoystickButton RightStickButton;

    public final POVButton povUp;
    public final POVButton povRight;
    public final POVButton povDown;
    public final POVButton povLeft;

    /**
     * Creates a new Xbox controller wrapper.
     * @param port The port number the controller is connected to
     */
    public XBoxWrapper(int port) {
        this.joystick = new Joystick(port);

        this.A = new JoystickButton(joystick, XBoxMapping.A);
        this.B = new JoystickButton(joystick, XBoxMapping.B);
        this.X = new JoystickButton(joystick, XBoxMapping.X);
        this.Y = new JoystickButton(joystick, XBoxMapping.Y);
        
        this.LB = new JoystickButton(joystick, XBoxMapping.LB);
        this.RB = new JoystickButton(joystick, XBoxMapping.RB);
        this.LeftStickButton = new JoystickButton(joystick, XBoxMapping.LeftStickButton);
        this.RightStickButton = new JoystickButton(joystick, XBoxMapping.RightStickButton);

        this.Start = new JoystickButton(joystick, XBoxMapping.Start);
        this.Back = new JoystickButton(joystick, XBoxMapping.Back);

        this.povUp = new POVButton(joystick, 0);
        this.povRight = new POVButton(joystick, 90);
        this.povDown = new POVButton(joystick, 180);
        this.povLeft = new POVButton(joystick, 270);
    }

    /**
     * Gets the X-axis value of the left stick.
     * @return Value from -1.0 to 1.0
     */
    public double getLeftX() {
        return joystick.getRawAxis(0);
    }

    /**
     * Gets the Y-axis value of the left stick.
     * @return Value from -1.0 to 1.0
     */
    public double getLeftY() {
        return joystick.getRawAxis(1);
    }

    /**
     * Gets the X-axis value of the right stick.
     * @return Value from -1.0 to 1.0
     */
    public double getRightX() {
        return joystick.getRawAxis(4);
    }

    /**
     * Gets the Y-axis value of the right stick.
     * @return Value from -1.0 to 1.0
     */
    public double getRightY() {
        return joystick.getRawAxis(5);
    }

    /**
     * Gets the value of the left trigger.
     * @return Value from 0.0 to 1.0
     */
    public double getLeftTrigger() {
        return joystick.getRawAxis(2);
    }

    /**
     * Gets the value of the right trigger.
     * @return Value from 0.0 to 1.0
     */
    public double getRightTrigger() {
        return joystick.getRawAxis(3);
    }
    /**
     * set the rumble of the controller
     * @return nil
     */
      public void setRumble(RumbleType type, double power) {
        joystick.setRumble(type, power);
    }
}
