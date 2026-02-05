package frc.robot.subsystems;

import ca.team4308.absolutelib.math.trajectories.*;
import ca.team4308.absolutelib.math.trajectories.flywheel.*;
import ca.team4308.absolutelib.math.trajectories.gamepiece.*;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Example shooter subsystem demonstrating the 2026 trajectory calculation
 * system. Uses FlywheelGenerator to find optimal shot configurations and
 * TrajectorySolver for real-time shot calculations.
 */
public class ExampleShooter extends AbsoluteSubsystem {

    // Hardware
    private final MotorWrapper flywheelLeader;

    // Trajectory System
    private final TrajectorySolver solver;
    private final FlywheelConfig flywheelConfig;
    private final GamePiece gamePiece;

    // State
    private double targetRpm = 0.0;
    private double targetPitchDegrees = 0.0;
    private double targetYawDegrees = 0.0;
    private boolean hasValidShot = false;
    private ShotCandidate lastShot = null;
    private TrajectoryResult lastTrajectoryResult = null;
    private Pose3d[] lastFlightPath = new Pose3d[0];
    
    // Pose supplier for continuous tracking
    private Supplier<Pose2d> poseSupplier = null;
    
    // Shooter mounting parameters
    private double shooterHeightMeters = 0.6; // Height of shooter on robot
    
    // Hood/pivot angle limits (degrees)
    private double minPitchDegrees = 0.0;
    private double maxPitchDegrees = 50.0;
    
    private Translation3d targetPosition = new Translation3d(5.0, 4.0, 2.1); // Default to blue alliance goal
    
    // Enable/disable continuous tracking
    private boolean trackingEnabled = false;
    
    // Debug: store distance for logging
    private double lastDistanceToTarget = 0.0;

    public ExampleShooter() {
        super();

        flywheelLeader = new MotorWrapper(MotorType.TALONFX, 40);
   

        gamePiece = GamePieces.REBUILT_2026_BALL;
        solver = TrajectorySolver.forGame2026();

        FlywheelGenerator generator = new FlywheelGenerator(gamePiece);
        FlywheelGenerator.GenerationResult result = generator.generateAndEvaluate(15.0);

        if (result.bestConfig != null) {
            flywheelConfig = result.bestConfig.config;
            System.out.println("Using flywheel config: " + flywheelConfig.getName());
        } else {
            // Fallback to a preset configuration
            flywheelConfig = FlywheelConfig.builder()
                    .name("Default Shooter")
                    .arrangement(FlywheelConfig.WheelArrangement.DUAL_OVER_UNDER)
                    .wheelDiameterInches(4.0)
                    .wheelWidthInches(2.0)
                    .material(WheelMaterial.VERY_HARD)
                    .compressionRatio(0.10)
                    .wheelCount(2)
                    .motor(ca.team4308.absolutelib.math.trajectories.motor.FRCMotors.KRAKEN_X60)
                    .motorsPerWheel(1)
                    .gearRatio(1.0)
                    .build();
        }
    }
    
    /**
     * Set the pose supplier for continuous trajectory tracking.
     * This allows the shooter to automatically update its trajectory calculation
     * based on the robot's current position.
     * 
     * @param supplier A supplier that returns the robot's current Pose2d
     */
    public void setPoseSupplier(Supplier<Pose2d> supplier) {
        this.poseSupplier = supplier;
    }
    
    /**
     * Set the target position (goal) for trajectory calculations.
     * 
     * @param x Target X position in meters
     * @param y Target Y position in meters  
     * @param z Target Z position (height) in meters
     */
    public void setTarget(double x, double y, double z) {
        this.targetPosition = new Translation3d(x, y, z);
    }
    
    /**
     * Set the shooter height on the robot.
     * 
     * @param heightMeters Height of shooter above ground in meters
     */
    public void setShooterHeight(double heightMeters) {
        this.shooterHeightMeters = heightMeters;
    }
    
    /**
     * Set the hood/pivot angle limits.
     * 
     * @param minDegrees Minimum pitch angle in degrees
     * @param maxDegrees Maximum pitch angle in degrees
     */
    public void setPitchLimits(double minDegrees, double maxDegrees) {
        this.minPitchDegrees = minDegrees;
        this.maxPitchDegrees = maxDegrees;
    }
    
    /**
     * Enable or disable continuous trajectory tracking.
     * When enabled, the shooter will continuously update its trajectory
     * calculation based on the robot's current position.
     * 
     * @param enabled True to enable tracking, false to disable
     */
    public void setTrackingEnabled(boolean enabled) {
        this.trackingEnabled = enabled;
    }
    
    /**
     * Check if tracking is currently enabled.
     */
    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    @Override
    public void periodic() {
        // Continuous tracking: update trajectory calculation every cycle if enabled
        if (trackingEnabled && poseSupplier != null) {
            Pose2d currentPose = poseSupplier.get();
            calculateShot(
                currentPose.getX(), currentPose.getY(), shooterHeightMeters,
                targetPosition.getX(), targetPosition.getY(), targetPosition.getZ()
            );
            // Log robot position for debugging
            recordOutput("RobotX", currentPose.getX());
            recordOutput("RobotY", currentPose.getY());
        }
        
        // Update motor control based on target RPM
        if (targetRpm > 0) {
            double percentOutput = Math.min(targetRpm / 6000.0, 1.0);
            flywheelLeader.set(percentOutput);
        }
        
        recordOutput("ValidShot", hasValidShot);
        recordOutput("TargetRPM", targetRpm);
        recordOutput("TargetPitchDegrees", targetPitchDegrees);
        recordOutput("TargetYawDegrees", targetYawDegrees);
        recordOutput("TrackingEnabled", trackingEnabled);
        recordOutput("DistanceToTarget", lastDistanceToTarget);
        recordOutput("TrajectoryPoints", lastFlightPath.length);
        
        Pose3d goalPose = new Pose3d(targetPosition, new Rotation3d());
        Logger.recordOutput("ExampleShooter/GoalPose3d", goalPose);
        Logger.recordOutput("ExampleShooter/GoalPose3dArray", new Pose3d[] { goalPose });
        
        recordOutput("TargetX", targetPosition.getX());
        recordOutput("TargetY", targetPosition.getY());
        recordOutput("TargetZ", targetPosition.getZ());
        
        if (lastFlightPath.length > 0) {
            Logger.recordOutput("ExampleShooter/Trajectory", lastFlightPath);
            Pose3d firstPoint = lastFlightPath[0];
            Pose3d lastPoint = lastFlightPath[lastFlightPath.length - 1];
            recordOutput("TrajectoryStartX", firstPoint.getX());
            recordOutput("TrajectoryStartY", firstPoint.getY());
            recordOutput("TrajectoryStartZ", firstPoint.getZ());
            recordOutput("TrajectoryEndX", lastPoint.getX());
            recordOutput("TrajectoryEndY", lastPoint.getY());
            recordOutput("TrajectoryEndZ", lastPoint.getZ());
        } else {
            Logger.recordOutput("ExampleShooter/Trajectory", new Pose3d[0]);
        }
        
        if (lastTrajectoryResult != null) {
            recordOutput("TrajectoryStatus", lastTrajectoryResult.getStatus().name());
            recordOutput("StatusMessage", lastTrajectoryResult.getStatusMessage());
        }
        
        if (lastShot != null) {
            recordOutput("TimeOfFlight", lastShot.getTimeOfFlightSeconds());
            recordOutput("AccuracyScore", lastShot.getAccuracyScore());
            recordOutput("MaxHeight", lastShot.getMaxHeightMeters());
            recordOutput("RequiredVelocity", lastShot.getRequiredVelocityMps());
            recordOutput("ArcType", lastShot.getArcType().name());
        }
        
        if (lastTrajectoryResult != null && lastTrajectoryResult.isSuccess()) {
            recordOutput("Confidence", lastTrajectoryResult.getConfidenceScore());
            recordOutput("MarginOfError", lastTrajectoryResult.getMarginOfErrorMeters());
        }
    }

    /**
     * Calculate shot parameters for a target position. Call this before
     * shooting to update targetRpm and targetPitchDegrees.
     *
     * @param shooterX Shooter X position on field (meters)
     * @param shooterY Shooter Y position on field (meters)
     * @param shooterZ Shooter height (meters)
     * @param targetX Target X position (meters)
     * @param targetY Target Y position (meters)
     * @param targetZ Target height (meters)
     */
    public void calculateShot(double shooterX, double shooterY, double shooterZ,
            double targetX, double targetY, double targetZ) {
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
        double yawToTargetRadians = Math.atan2(dy, dx);
        lastDistanceToTarget = Math.sqrt(dx * dx + dy * dy);
        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterZ)
                .shooterYawRadians(yawToTargetRadians) // Set yaw to point at target
                .targetPositionMeters(targetX, targetY, targetZ)
                .pitchRangeDegrees(minPitchDegrees, maxPitchDegrees) // Hood limits
                .shotPreference(ShotInput.ShotPreference.MOST_ACCURATE)
                .build();

        lastTrajectoryResult = solver.solve(input);
        
        if (lastTrajectoryResult.isSuccess()) {
            List<Pose3d> flightPathList = lastTrajectoryResult.getFlightPath();
            
            lastFlightPath = flightPathList.toArray(new Pose3d[0]);
            
            targetPitchDegrees = lastTrajectoryResult.getPitchAngleDegrees();
            targetYawDegrees = Math.toDegrees(yawToTargetRadians);
            targetRpm = lastTrajectoryResult.getRecommendedRpm();
            hasValidShot = true;
            
            lastShot = ShotCandidate.builder()
                    .pitchAngleRadians(lastTrajectoryResult.getPitchAngleRadians())
                    .requiredVelocityMps(lastTrajectoryResult.getRequiredVelocityMps())
                    .yawAngleRadians(lastTrajectoryResult.getYawAdjustmentRadians())
                    .timeOfFlightSeconds(lastTrajectoryResult.getTimeOfFlightSeconds())
                    .maxHeightMeters(lastTrajectoryResult.getMaxHeightMeters())
                    .closestApproachMeters(lastTrajectoryResult.getMarginOfErrorMeters())
                    .hitsTarget(true)
                    .accuracyScore(lastTrajectoryResult.getConfidenceScore())
                    .build();
        } else {
            ShotCandidateList candidates = solver.findAllCandidates(input);
            Optional<ShotCandidate> fastest = candidates.getMostAccurate();
            
            if (fastest.isPresent()) {
                lastShot = fastest.get();
                targetPitchDegrees = lastShot.getPitchAngleDegrees();
                targetYawDegrees = lastShot.getYawAngleDegrees();
                targetRpm = lastShot.getSpeedScore();
                hasValidShot = true;
                lastFlightPath = new Pose3d[0]; // No trajectory available from candidates
            } else {
                hasValidShot = false;
                targetRpm = 0;
                targetPitchDegrees = 0;
                targetYawDegrees = 0;
                lastFlightPath = new Pose3d[0];
            }
        }
    }

    /**
     * Command to calculate and prepare a shot.
     */
    public Command prepareShot(double shooterX, double shooterY, double shooterZ,
            double targetX, double targetY, double targetZ) {
        return runOnce(() -> calculateShot(shooterX, shooterY, shooterZ, targetX, targetY, targetZ));
    }

    /**
     * Command to spin up the flywheel to target RPM.
     */
    public Command spinUp() {
        return run(() -> {
            if (hasValidShot && targetRpm > 0) {
                double percentOutput = Math.min(targetRpm / 6000.0, 1.0);
                flywheelLeader.set(percentOutput);
            }
        });
    }

    /**
     * Stop the shooter.
     */
    public void stop() {
        targetRpm = 0;
        flywheelLeader.stop();
    }

    /**
     * Command to stop the shooter.
     */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    // Getters for state
    public double getTargetRpm() {
        return targetRpm;
    }

    public double getTargetPitchDegrees() {
        return targetPitchDegrees;
    }
    
    public double getTargetYawDegrees() {
        return targetYawDegrees;
    }

    public boolean hasValidShot() {
        return hasValidShot;
    }

    public ShotCandidate getLastShot() {
        return lastShot;
    }
    
    /**
     * Returns the last calculated trajectory result.
     * Contains all trajectory information including flight path, confidence, etc.
     */
    public TrajectoryResult getLastTrajectoryResult() {
        return lastTrajectoryResult;
    }
    
    /**
     * Returns the flight path as an array of Pose3d for visualization.
     * Each Pose3d contains position (x, y, z) and orientation based on velocity vector.
     * Useful for AdvantageScope 3D visualization.
     * 
     * @return Array of Pose3d representing the ball's trajectory, empty if no valid shot
     */
    public Pose3d[] getFlightPath() {
        return lastFlightPath;
    }
    
    /**
     * Returns the flight path as a List of Pose3d.
     * Same as getFlightPath() but as a List for convenience.
     * 
     * @return List of Pose3d representing the ball's trajectory
     */
    public List<Pose3d> getFlightPathList() {
        return List.of(lastFlightPath);
    }
    
    /**
     * Returns the number of trajectory points in the flight path.
     */
    public int getTrajectoryPointCount() {
        return lastFlightPath.length;
    }
    
    /**
     * Returns the estimated time of flight in seconds.
     * @return Time of flight, or 0 if no valid shot
     */
    public double getTimeOfFlight() {
        return lastShot != null ? lastShot.getTimeOfFlightSeconds() : 0.0;
    }
    
    /**
     * Returns the maximum height the ball will reach during flight.
     * @return Max height in meters, or 0 if no valid shot
     */
    public double getMaxHeight() {
        return lastShot != null ? lastShot.getMaxHeightMeters() : 0.0;
    }
    
    /**
     * Returns the required exit velocity for the shot.
     * @return Exit velocity in m/s, or 0 if no valid shot
     */
    public double getRequiredVelocity() {
        return lastShot != null ? lastShot.getRequiredVelocityMps() : 0.0;
    }

    public FlywheelConfig getFlywheelConfig() {
        return flywheelConfig;
    }


    
    @Override
    public Sendable log() {
        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("ExampleShooter");
                builder.addDoubleProperty("TargetRPM", () -> targetRpm, null);
                builder.addDoubleProperty("TargetPitchDeg", () -> targetPitchDegrees, null);
                builder.addDoubleProperty("TargetYawDeg", () -> targetYawDegrees, null);
                builder.addBooleanProperty("HasValidShot", () -> hasValidShot, null);
                builder.addIntegerProperty("TrajectoryPoints", () -> lastFlightPath.length, null);
                
                // Shot metrics
                builder.addDoubleProperty("TimeOfFlight", () -> lastShot != null ? lastShot.getTimeOfFlightSeconds() : 0.0, null);
                builder.addDoubleProperty("MaxHeight", () -> lastShot != null ? lastShot.getMaxHeightMeters() : 0.0, null);
                builder.addDoubleProperty("RequiredVelocity", () -> lastShot != null ? lastShot.getRequiredVelocityMps() : 0.0, null);
                builder.addDoubleProperty("Confidence", () -> lastShot != null ? lastShot.getAccuracyScore() : 0.0, null);
                
                // Trajectory result specific data
                builder.addDoubleProperty("MarginOfError", () -> lastTrajectoryResult != null && lastTrajectoryResult.isSuccess() 
                        ? lastTrajectoryResult.getMarginOfErrorMeters() : 0.0, null);
            }
        };
    }
}
