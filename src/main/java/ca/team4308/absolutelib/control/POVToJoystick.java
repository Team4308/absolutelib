package ca.team4308.absolutelib.control;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// Added helper import (optional future use)
import java.util.function.BooleanSupplier;

/**
 * A {@link Trigger} that gets its state from a {@link GenericHID}.
 * <p>
 * Matches POV angle against a specified POV index on the HID.
 * NOTE: Null check currently occurs after super() which means if a null joystick
 * were passed, the supplier would capture null before validation. Consider moving
 * requireNonNullParam before super() in a future revision.
 */
public class POVToJoystick extends Trigger {
    /**
     * Creates a joystick POV "button" for triggering commands.
     *
     * @param joystick     The GenericHID object that has the POV (e.g. Joystick)
     * @param angle        The POV angle being matched (e.g. 0, 90, 180, 270)
     * @param povNumber    The POV index (usually 0). Previous comment referenced buttonNumber; kept for compatibility.
     *
     * Implementation note:
     *  - The lambda checks joystick.getPOV(povNumber) == angle.
     *  - Null validation runs after super(); safe in normal usage but could be moved earlier.
     */
    public POVToJoystick(GenericHID joystick, int angle, int povNumber) {
        super(() -> joystick.getPOV(povNumber) == angle); // existing logic preserved
        requireNonNullParam(joystick, "joystick", "POVButton"); // original validation retained
        // Additional diagnostic comment:
        // If joystick is null, a NullPointerException would occur inside the Trigger evaluation before this check.
    }

    /**
     * Creates a POV trigger for POV index 0.
     *
     * @param joystick The GenericHID object that has the POV
     * @param angle    Desired POV angle
     */
    public POVToJoystick(GenericHID joystick, int angle) {
        this(joystick, angle, 0);
    }

    @SuppressWarnings("unused")
    private static BooleanSupplier povCondition(GenericHID joystick, int angle, int povNumber) {
        requireNonNullParam(joystick, "joystick", "POVToJoystick");
        return () -> joystick.getPOV(povNumber) == angle;
    }
}