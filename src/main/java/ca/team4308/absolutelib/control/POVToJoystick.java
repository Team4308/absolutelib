package ca.team4308.absolutelib.control;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A {@link Trigger} that gets its state from a {@link GenericHID}.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class POVToJoystick extends Trigger {
    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick,
     *                     KinectStick, etc)
     * @param buttonNumber The button number (see
     *                     {@link GenericHID#getRawButton(int) }
     */
    public POVToJoystick(GenericHID joystick, int angle, int povNumber) {
        super(() -> joystick.getPOV(povNumber) == angle);
        requireNonNullParam(joystick, "joystick", "POVButton");
    }

    /**
     * Creates a POV button for triggering commands. By default, acts on POV 0
     *
     * @param joystick The GenericHID object that has the POV
     * @param angle    The desired angle (e.g. 90, 270)
     */
    public POVToJoystick(GenericHID joystick, int angle) {
        this(joystick, angle, 0);
    }
}