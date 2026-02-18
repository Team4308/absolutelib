// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Util.FuelSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

    // Subsystems - Only Swerve, Shooter, Pivot (for shooter angle), and LEDs
    private final ExamplePivot m_pivot = new ExamplePivot();
    private final ExampleLEDs m_leds = new ExampleLEDs();
    private final ExampleShooter m_shooter = new ExampleShooter();
    private final FuelSim m_FuelSim = FuelSim.getInstance();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    private final XBoxWrapper driver = new XBoxWrapper(1);

    private final SendableChooser<Command> autoChooser;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driver.getLeftY() * -1,
            () -> driver.getLeftX() * -1)
            .withControllerRotationAxis(() -> driver.getRightX() * -1)
            .deadband(Constants.OperatorConstants.DEADBAND)
            .scaleTranslation(1.0)
            .allianceRelativeControl(true);

    // Clone's the angular velocity input stream and converts it to a fieldRelative
    // input stream.
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
            driver::getRightY)
            .headingWhile(true);

    // Clone's the angular velocity input stream and converts it to a roboRelative
    // input stream.
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX())
            .withControllerRotationAxis(() -> driver.getRightX())
            .deadband(Constants.OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                    driver.getLeftTrigger() * Math.PI) * (Math.PI * 2),
                    () -> Math.cos(driver.getLeftTrigger() * Math.PI) * (Math.PI * 2))
            .headingWhile(true);

    public RobotContainer() {
        m_shooter.setPoseSupplier(drivebase::getPose);
        m_shooter.setChassisSpeedsSupplier(drivebase::getFieldVelocity); 
        m_shooter.setShooterHeight(0.6);
        m_shooter.setPitchLimits(47.5, 82.5);
        m_shooter.setTrackingEnabled(true); 
        updateTargetForAlliance();
        configureBindings();
        configureNamedCommands();
        initFuelSim();
        DriverStation.silenceJoystickConnectionWarning(true);
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        m_leds.setIdle();

    }


    public void updateTargetForAlliance() {
        var alliance = DriverStation.getAlliance();
        double goalZ = 2.1; 
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            m_shooter.setTarget(12.0, 4.0, goalZ);
        } else {
            m_shooter.setTarget(4.5, 4.0, goalZ);
        }
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);


        driver.RB.whileTrue(
                Commands.run(() -> {
                    if (m_shooter.hasValidShot()) {
                        m_pivot.setAngle(m_shooter.getTargetPitchDegrees()).schedule();
                        m_leds.setProgress(0.5, edu.wpi.first.wpilibj.util.Color.kYellow);
                    }
                }));

        driver.LB.whileTrue(m_shooter.spinUp());

        driver.RightStickButton.whileTrue(
                Commands.run(() -> {
                    if (m_shooter.hasValidShot()) {
                        m_pivot.setAngle(m_shooter.getTargetPitchDegrees()).schedule();
                    }
                }).alongWith(m_shooter.spinUp()));

        driver.LeftStickButton.whileTrue(
                m_shooter.stopCommand()
                        .andThen(m_pivot.setAngle(0.0))
                        .andThen(Commands.runOnce(() -> m_leds.setIdle())));

        driver.A.onTrue(Commands.runOnce(() -> {
            m_shooter.setTrackingEnabled(!m_shooter.isTrackingEnabled());
            if (m_shooter.isTrackingEnabled()) {
                m_leds.setProgress(0.3, edu.wpi.first.wpilibj.util.Color.kGreen);
            } else {
                m_leds.setIdle();
            }
        }));

        // B button: cycle shot mode (LOOKUP_ONLY → SOLVER_ONLY → LOOKUP_WITH_SOLVER_FALLBACK → ...)
        driver.B.onTrue(m_shooter.cycleModeCommand()
                .andThen(Commands.runOnce(() -> 
                        m_leds.setProgress(0.7, edu.wpi.first.wpilibj.util.Color.kCyan))));

        driver.Start.onTrue(
                m_shooter.spinUp().withTimeout(3.0)
                        .andThen(Commands.runOnce(() -> m_leds.setSuccess())));

        driver.Back.onTrue(
                m_shooter.stopCommand()
                        .andThen(Commands.runOnce(() -> m_leds.setError())));

        driver.povUp.onTrue(m_shooter.shootBallSimCommand()); // Shoot ball in simulation
        driver.povDown.onTrue(Commands.runOnce(() -> m_leds.setAlliance()));
        driver.povLeft.onTrue(Commands.runOnce(() -> m_leds.setRainbow()));
        driver.povRight.onTrue(Commands.runOnce(() -> {
            if (m_shooter.hasValidShot())
                m_leds.setSuccess();
            else
                m_leds.setError();
        }));

        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
        } else {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }
    }

    public void configureNamedCommands() {
    }

    

    /**
     * Returns the autonomous command to run.
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> m_shooter.calculateShot(2.0, 4.0, 0.6, 8.0, 4.0, 2.1)),
                m_pivot.setAngle(35.0),
                Commands.waitSeconds(0.3),
                m_shooter.spinUp().withTimeout(2.0),
                Commands.runOnce(() -> m_leds.setSuccess()),
                Commands.waitSeconds(0.5),
                m_shooter.stopCommand(),
                m_pivot.setAngle(0.0));
    }


    private void initFuelSim() {
        FuelSim.getInstance(); // gets singleton instance of FuelSim
        //FuelSim.getInstance().spawnStartingFuel(); // spawns fuel in the depots and neutral zone

        // Register a robot for collision with fuel
        FuelSim.getInstance().registerRobot(
                Units.inchesToMeters(34), // from left to right
                Units.inchesToMeters(34), // from front to back
                Units.inchesToMeters(6), // from floor to top of bumpers
                drivebase::getPose, // Supplier<Pose2d> of robot pose
                drivebase::getFieldVelocity); // Supplier<ChassisSpeeds> of field-centric chassis speeds

        FuelSim.getInstance().setSubticks(5); // sets the number of physics iterations to perform per 20ms loop. Default
                                              // = 5

        FuelSim.getInstance().start();
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
