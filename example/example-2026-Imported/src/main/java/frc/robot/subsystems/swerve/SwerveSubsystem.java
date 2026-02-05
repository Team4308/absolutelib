package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.PoseCamera;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private PIDConstants ANGLE_CONTROLLER;
  private PIDConstants TRANSLATION_CONTROLLER;

  private PPHolonomicDriveController ALIGN_CONTROLLER;

  private final SwerveDrive swerveDrive;

  private final boolean usingVision = false;
  private PoseCamera poseCameras;

  private Pose2d targetPose = new Pose2d();

  private Field2d m_driverStationField = new Field2d();

  private double resetTime = Timer.getFPGATimestamp() + 2; // To clear DS field 2 seconds after boot. Why? Idk

  { // To clear DS field 2 seconds after boot. Why? Idk
    new Thread(() -> {
      while (Timer.getFPGATimestamp() < resetTime) {
        Thread.yield();
      }
      clearDriverField();
    }).start();
  }

  // Puts the pathing onto DS
  private final DoubleArraySubscriber pathPointsSub = NetworkTableInstance.getDefault()
      .getTable("AdvantageKit/LocalADStarAK")
      .getDoubleArrayTopic("CurrentPathPoints").subscribe(new double[] {});

  private final BooleanSubscriber newPathSub = NetworkTableInstance.getDefault()
      .getTable("AdvantageKit/LocalADStarAK")
      .getBooleanTopic("IsNewPathAvailable")
      .subscribe(false);

  private double alignedStartTime = -1; // Tracks how long the bot has been aligned for

  public SwerveSubsystem(File directory) {
    if (Robot.isSimulation()) // Removes Ramp so we can just drive over it in sim
      SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
    try {
      swerveDrive = new SwerveParser(directory)
          .createSwerveDrive(
              Constants.MAX_SPEED,
              new Pose2d(
                  new Translation2d(Meter.of(2), Meter.of(4)), Rotation2d.fromDegrees(0))); // TODO: Set starting pose
                                                                                            // correctly
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Only set to true if controlling the robot via angle.
    swerveDrive.setCosineCompensator(
        false); // Correct for skew that gets worse as angular velocity increases. Start with a
                // coefficient of 0.1. Could be negative
    swerveDrive.setAngularVelocityCompensation(
        false, false, 0.1); // Resync absolute encoders and motor encoders periodically when they are not
                            // moving.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    if (usingVision) {
      setupPhotonVision();
      swerveDrive.stopOdometryThread(); // Stop the odometry thread while running vision to synchronize updates better.
    }
    ANGLE_CONTROLLER = Constants.Swerve.Heading.HEADING_PID;
    TRANSLATION_CONTROLLER = Constants.Swerve.Translation.TRANSLATION_PID;

    ALIGN_CONTROLLER = new PPHolonomicDriveController(
        Constants.Swerve.Translation.TRANSLATION_PID, Constants.Swerve.Heading.HEADING_PID);

    setupPathPlanner();
  }

  public void setupPhotonVision() {
    poseCameras = new PoseCamera(swerveDrive::getPose, swerveDrive.field);
  }

  @Override
  public void periodic() {
    if (usingVision) {
      swerveDrive.updateOdometry();
      poseCameras.updatePoseEstimation(swerveDrive);

      poseCameras.updateVisionField();
    }

    // Puts the pathing onto DS
    if (newPathSub.get()) {
      double[] pointArray = pathPointsSub.get();
      int numPoses = pointArray.length / 2;
      Pose2d[] poses = new Pose2d[numPoses];

      for (int i = 0; i < numPoses; i++) {
        poses[i] = new Pose2d(
            pointArray[i * 2],
            pointArray[i * 2 + 1],
            new Rotation2d());
      }

      Logger.recordOutput("Swerve/Path", poses);
      m_driverStationField.getObject("Path").setPoses(poses);
    }

    m_driverStationField.setRobotPose(getPose());
    SmartDashboard.putData("DriverStationField", m_driverStationField);

    Logger.recordOutput("Swerve/Is Aligned?", isAligned());
    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/Velocity", getRobotVelocity());
  }

  private void clearDriverField() {
    Logger.recordOutput("Swerve/Path", new Pose2d[0]);
    m_driverStationField.getObject("Path").setPoses(new ArrayList<>());
  }

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably store this in
    // your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry, // called if auto has starting pose
          this::getRobotVelocity, // must be robot relative
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
              // controller for holonomic drive trains
              // Translation PID constants
              TRANSLATION_CONTROLLER,
              // Rotation PID constants
              ANGLE_CONTROLLER),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      e.printStackTrace();

    }
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /**
   * Checks if the bot is aligned translation-wise
   * 
   * @return boolean aligned
   */
  public boolean isTranslationAligned() {
    Translation2d currentTranslation2d = getPose().getTranslation();
    Translation2d targetTranslation2d = targetPose.getTranslation();

    return currentTranslation2d.getDistance(targetTranslation2d) < Constants.Swerve.Translation.TOLERANCE;
  }

  /**
   * Checks if the bot is aligned heading-wise
   * 
   * @return boolean aligned
   */
  public boolean isHeadingAligned() {
    Rotation2d currentHeading = getPose().getRotation();
    Rotation2d targetHeading = targetPose.getRotation();

    double angleDelta = currentHeading.minus(targetHeading).getDegrees();

    boolean headingAligned = angleDelta < Constants.Swerve.Heading.TOLERANCE;

    return headingAligned;
  }

  /**
   * Checks if the bot is aligned heading-wise and translation-wise
   * 
   * @return
   */
  public boolean isAligned() {
    return isTranslationAligned() && isHeadingAligned();
  }

  /**
   * Checks if the bot is aligned for a period of time
   * 
   * @param holdTimeMillis How long the bot must stay aligned for
   * @return boolean aligned
   */
  public boolean isAligned(double holdTimeMillis) {
    if (isTranslationAligned() && isHeadingAligned()) {
      if (alignedStartTime == -1) {
        alignedStartTime = System.currentTimeMillis();
      } else if (System.currentTimeMillis() - alignedStartTime >= holdTimeMillis) {
        return true;
      }
    } else {
      alignedStartTime = -1;
    }
    return false;
  }

  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPoseObjAvoid(Supplier<Pose2d> pose) {
    return defer(() -> driveToPoseObjAvoid(pose.get()));
  }

  public Command driveToPoseObjAvoid(Pose2d pose) {
    targetPose = pose; // Sets the global target pose

    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        Constants.Swerve.PathFinding.constraints,
        0.0);

    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalState.pose = targetPose;
    Logger.recordOutput("Swerve/Path Goal", targetPose);
    Logger.recordOutput(
        "Swerve/PID output", ALIGN_CONTROLLER.calculateRobotRelativeSpeeds(getPose(), goalState));

    // Switches to PID once it is close enough
    return Commands.sequence(
        pathfindingCommand.until(() -> targetPose.getTranslation()
            .getDistance(getPose().getTranslation()) < Constants.Swerve.PathFinding.PIDTolerance),
        run(() -> {
          swerveDrive.drive(
              ALIGN_CONTROLLER.calculateRobotRelativeSpeeds(
                  getPose(), goalState));
          Logger.recordOutput(
              "Swerve/PID output", ALIGN_CONTROLLER.calculateRobotRelativeSpeeds(getPose(), goalState));
        }).until(() -> isAligned(200)))
        .finallyDo(() -> clearDriverField());

  }

  public Command driveToPose(Supplier<Pose2d> pose) {
    return defer(() -> driveToPose(pose.get()));
  }

  public Command driveToPose(Pose2d pose) { // Tried and tested Auto Align we used in 2025
    // Change global target pose
    targetPose = pose;

    // Create the goal state
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalState.pose = pose;

    // Generates a list of waypoints to follow
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(
            swerveDrive.getPose().getTranslation(),
            new Rotation2d()),
        pose);
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        Constants.Swerve.PathFinding.constraints,
        new IdealStartingState(
            MetersPerSecond.of(
                new Translation2d(
                    getFieldVelocity().vxMetersPerSecond,
                    getFieldVelocity().vyMetersPerSecond)
                    .getNorm()),
            getHeading()),
        new GoalEndState(0.0, pose.getRotation()));
    path.preventFlipping = true;

    Logger.recordOutput("Swerve/Path Goal", pose);

    // Visualize Path on driver station
    ArrayList<Pose2d> points = new ArrayList<>();
    for (PathPoint state : path.getAllPathPoints()) {
      points.add(new Pose2d(state.position, new Rotation2d(0)));
    }

    m_driverStationField.getObject("Path").setPoses(points);
    Logger.recordOutput("Swerve/Path", points.toArray(new Pose2d[0]));

    // Runs until ~10 inches to target, then switches to pid
    return AutoBuilder.followPath(path)
        .until(() -> pose.getTranslation()
            .getDistance(getPose().getTranslation()) < Constants.Swerve.PathFinding.PIDTolerance)
        .andThen(
            run(
                () -> {
                  swerveDrive.drive(
                      ALIGN_CONTROLLER.calculateRobotRelativeSpeeds(getPose(), goalState));
                  Logger.recordOutput(
                      "Swerve/PID output", ALIGN_CONTROLLER.calculateRobotRelativeSpeeds(getPose(), goalState));
                }))
        .finallyDo(interrupted -> {
          clearDriverField();
        }).until(() -> isAligned(500));
  }

  /**
   * Drives a certain distance at a certain speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(
            () -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceInMeters);
  }

  /*
   * //TODO: IMPLEMENT OBJECT CAMERA
   * public Command aimAtTarget(Cameras camera) {
   * return run(() -> {
   * Optional<PhotonPipelineResult> resultO = camera.getBestResult();
   * if (resultO.isPresent()) {
   * var result = resultO.get();
   * if (result.hasTargets()) {
   * drive(getTargetSpeeds(0,
   * 0,
   * Rotation2d.fromDegrees(result.getBestTarget()
   * .getYaw()))); // Not sure if this will work, more math may be required.
   * }
   * }
   * });
   * }
   */

  // SysID Drive Motors Characterization
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
        3.0,
        5.0,
        3.0);
  }

  // SysID Angle Motors Characterization
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
  }

  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  public void setRobotPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  // Command to drive the robot using translative values and heading as angular
  // velocity.
  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move
          swerveDrive.drive(
              SwerveMath.scaleTranslation(
                  new Translation2d(
                      translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                      translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                  0.8),
              Math.pow(angularRotationX.getAsDouble(), 3)
                  * swerveDrive.getMaximumChassisAngularVelocity(),
              true,
              false);
        });
  }

  // Command to drive the robot using translative values and heading as a
  // setpoint.
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(
        () -> {
          Translation2d scaledInputs = SwerveMath.scaleTranslation(
              new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  // Primary method for controlling the drivebase, for which the swerve drive
  // methods base on.
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used
    // most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  // Resets the gyro angle to zero and resets odometry to the same position, but
  // facing toward 0.
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  // Checks if alliance is red, false if blue or not available.
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  // If red alliance rotate the robot 180 after the drivebase zero command
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot
   * at an offset of 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  // Gets the current field-relative velocity (x, y and omega) of the robot
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  // Gets the current velocity (x, y and omega) of the robot
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  // Lock the swerve drive to prevent it from moving.
  public void lock() {
    swerveDrive.lockPose();
  }

  // Gets the current pitch angle of the robot, as reported by the imu.
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
}
