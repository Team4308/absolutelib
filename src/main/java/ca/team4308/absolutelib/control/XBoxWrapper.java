package ca.team4308.absolutelib.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class XBoxWrapper {
    public final Joystick joystick;

    public final JoystickButton A;
	public final JoystickButton B;
	public final JoystickButton X;
	public final JoystickButton Y;
	
	public final JoystickButton LB;
    public final JoystickButton RB;

	public final JoystickButton Start;
    public final JoystickButton Back;

    public XBoxWrapper(int port) {
        this.joystick = new Joystick(port);

        this.A = new JoystickButton(joystick, 1);
        this.B = new JoystickButton(joystick, 2);
        this.X = new JoystickButton(joystick, 3);
        this.Y = new JoystickButton(joystick, 4);

        this.LB = new JoystickButton(joystick, 5);
        this.RB = new JoystickButton(joystick, 6);

        this.Start = new JoystickButton(joystick, 8);
        this.Back = new JoystickButton(joystick, 7);
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