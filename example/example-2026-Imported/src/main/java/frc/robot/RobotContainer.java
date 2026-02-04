// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * RobotContainer demonstrates all AbsoluteLib subsystems with controller
 * bindings.
 *
 * Controller Layout: - A/Y: Elevator control - X/B: Pivot control - Bumpers:
 * Arm IK movement - Triggers: End effector (intake/outtake) - D-Pad: LED
 * pattern switching - Start: Calculate and spin up shooter
 */
public class RobotContainer {

    // Subsystems
    private final ExampleElevator m_elevator = new ExampleElevator();
    private final ExamplePivot m_pivot = new ExamplePivot();
    private final ExampleArm m_arm = new ExampleArm();
    private final ExampleEndEffector m_endEffector = new ExampleEndEffector();
    private final ExampleLEDs m_leds = new ExampleLEDs();
    private final ExampleShooter m_shooter = new ExampleShooter();
    // Note: ExampleVision requires SwerveDrive instance, uncomment when integrated:
    // private final ExampleVision m_vision;

    private final CommandXboxController m_driverController
            = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();

        // Initialize LEDs to idle pattern
        m_leds.setIdle();
    }

    private void configureBindings() {
        // ==================== Elevator Control (A/Y) ====================
        m_driverController.a().onTrue(m_elevator.moveToHeight(0.0));      // Ground
        m_driverController.y().onTrue(m_elevator.moveToHeight(1.0));      // High

        // ==================== Pivot Control (X/B) ====================
        m_driverController.x().onTrue(m_pivot.setAngle(-45.0));           // Retracted
        m_driverController.b().onTrue(m_pivot.setAngle(45.0));            // Extended

        // ==================== Arm Control (Bumpers) - IK movement ====================
        m_driverController.leftBumper().onTrue(m_arm.moveToPoint(0.5, 0.5));   // Stow position
        m_driverController.rightBumper().onTrue(m_arm.moveToPoint(1.0, 0.0));  // Extended position

        // ==================== End Effector (Triggers) ====================
        // Right trigger: Run intake while held
        m_driverController.rightTrigger(0.5).whileTrue(
                Commands.run(() -> m_endEffector.intake(), m_endEffector)
        );

        // Left trigger: Run outtake while held
        m_driverController.leftTrigger(0.5).whileTrue(
                Commands.run(() -> m_endEffector.outtake(), m_endEffector)
        ).onFalse(m_endEffector.stopCommand());

        // ==================== LED Patterns (D-Pad) ====================
        m_driverController.povUp().onTrue(Commands.runOnce(() -> m_leds.setRainbow()));
        m_driverController.povDown().onTrue(Commands.runOnce(() -> m_leds.setAlliance()));
        m_driverController.povLeft().onTrue(Commands.runOnce(() -> m_leds.setSuccess()));
        m_driverController.povRight().onTrue(Commands.runOnce(() -> m_leds.setError()));

        // ==================== Shooter (Start button) ====================
        // Calculate shot and spin up
        m_driverController.start().onTrue(
                m_shooter.prepareShot(2.0, 3.0, 0.6, 8.0, 6.0, 2.5)
                        .andThen(m_shooter.spinUp().withTimeout(3.0))
                        .andThen(Commands.runOnce(() -> m_leds.setSuccess()))
        );

        // Stop shooter on back button
        m_driverController.back().onTrue(m_shooter.stopCommand());
    }

    /**
     * Returns the autonomous command to run.
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
                // Simple auto: raise elevator, pivot out, then score
                m_elevator.moveToHeight(0.5),
                m_pivot.setAngle(30.0),
                Commands.waitSeconds(0.5),
                m_endEffector.outtake(),
                Commands.waitSeconds(1.0),
                m_endEffector.stopCommand(),
                m_elevator.moveToHeight(0.0),
                m_pivot.setAngle(0.0)
        );
    }

    /**
     * Update LED patterns based on robot state. Call this in robotPeriodic.
     */
    public void updateLEDs() {
        if (m_shooter.hasValidShot()) {
            m_leds.setProgress(m_shooter.getTargetRpm() / 6000.0, Color.kGreen);
        }
    }
}
