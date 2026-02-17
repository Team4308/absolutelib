package ca.team4308.absolutelib.control;

import ca.team4308.absolutelib.control.XBoxWrapper.XBoxMapping;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RazerWrapper {
  public static class RazerMapping {
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

  public final JoystickButton M1;
  public final JoystickButton M2;

  public final JoystickButton LeftStickButton;
  public final JoystickButton RightStickButton;

  public final POVToJoystick M3;
  public final POVToJoystick M4;
  public final POVToJoystick M5;
  public final POVToJoystick M6;

  public final Trigger RightTrigger;
  public final Trigger LeftTrigger;

  public final JoystickButton Start;
  public final JoystickButton Back;

  public final POVButton povUp;
  public final POVButton povRight;
  public final POVButton povDown;
  public final POVButton povLeft;

  public RazerWrapper(int port) {
    /*
     * requires: m1 to be start
     * m2 to be back
     * m3 to be povup
     * m4 to be povdown
     * m5 to be povleft
     * m6 to be povright
     *
     * these buttons will do the same thing, but should not be used anyways
     */

    this.joystick = new Joystick(port);

    this.A = new JoystickButton(joystick, RazerMapping.A);
    this.B = new JoystickButton(joystick, RazerMapping.B);
    this.X = new JoystickButton(joystick, RazerMapping.X);
    this.Y = new JoystickButton(joystick, RazerMapping.Y);

    this.LB = new JoystickButton(joystick, RazerMapping.LB);
    this.RB = new JoystickButton(joystick, RazerMapping.RB);
    this.LeftStickButton = new JoystickButton(joystick, RazerMapping.LeftStickButton);
    this.RightStickButton = new JoystickButton(joystick, RazerMapping.RightStickButton);

    this.M1 = new JoystickButton(joystick, RazerMapping.Start);
    this.M2 = new JoystickButton(joystick, RazerMapping.Back);

    this.M3 = new POVToJoystick(joystick, 0);
    this.M4 = new POVToJoystick(joystick, 180);
    this.M5 = new POVToJoystick(joystick, 270);
    this.M6 = new POVToJoystick(joystick, 90);

    this.RightTrigger = new Trigger(() -> getRightTriggerAsBoolean());
    this.LeftTrigger = new Trigger(() -> getLeftTriggerAsBoolean());

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

  public boolean getLeftTriggerAsBoolean() {
    return (joystick.getRawAxis(2) > 0.5);
  }

  public boolean getRightTriggerAsBoolean() {
    return (joystick.getRawAxis(3) > 0.5);
  }

  public boolean getLeftTriggerAsBoolean(double threshold) {
    return (joystick.getRawAxis(2) > threshold);
  }

  public boolean getRightTriggerAsBoolean(double threshold) {
    return (joystick.getRawAxis(3) > threshold);
  }

  public void setRumble(RumbleType type, double power) {
    joystick.setRumble(type, power);
  }
}
