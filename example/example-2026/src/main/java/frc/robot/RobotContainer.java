// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleArm;
import frc.robot.subsystems.ExampleElevator;
import frc.robot.subsystems.ExamplePivot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Subsystems

    private final ExampleElevator m_elevator = new ExampleElevator();
    private final ExamplePivot m_pivot = new ExamplePivot();
    private final ExampleArm m_arm = new ExampleArm();

    private final CommandXboxController m_driverController
            = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Elevator Control (A/Y)
        m_driverController.a().onTrue(m_elevator.moveToHeight(0.0));
        m_driverController.y().onTrue(m_elevator.moveToHeight(1.0));

        // Pivot Control (X/B)
        m_driverController.x().onTrue(m_pivot.setAngle(-45.0));
        m_driverController.b().onTrue(m_pivot.setAngle(45.0));

        // Arm Control (Bumpers) - IK to point
        m_driverController.leftBumper().onTrue(m_arm.moveToPoint(0.5, 0.5));
        m_driverController.rightBumper().onTrue(m_arm.moveToPoint(1.0, 0.0));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No auto configured");
    }
}
