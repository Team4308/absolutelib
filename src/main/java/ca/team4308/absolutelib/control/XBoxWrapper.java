package ca.team4308.absolutelib.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class XBoxWrapper {
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

    public double getLeftX() {
        return joystick.getRawAxis(0);
    }

    public double getLeftY() {
        return joystick.getRawAxis(1);
    }

    public double getRightX() {
        return joystick.getRawAxis(4);
    }

    public double getRightY() {
        return joystick.getRawAxis(5);
    }

    public double getLeftTrigger() {
        return joystick.getRawAxis(2);
    }

    public double getRightTrigger() {
        return joystick.getRawAxis(3);
    }
}
