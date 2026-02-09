package frc.robot.subsystems;

import ca.team4308.absolutelib.math.trajectories.*;
import ca.team4308.absolutelib.math.trajectories.flywheel.*;
import ca.team4308.absolutelib.math.trajectories.gamepiece.*;
import ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import frc.robot.Util.FuelSim;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Example shooter subsystem demonstrating the 2026 trajectory calculation
 * system. Uses FlywheelGenerator to find optimal shot configurations and
 * TrajectorySolver for real-time shot calculations.
 */
public class ExampleShooter extends AbsoluteSubsystem {

    private final MotorWrapper flywheelLeader;
    private final TrajectorySolver solver;
    private final FlywheelConfig flywheelConfig;
    private final GamePiece gamePiece;

    private double targetRpm = 0.0;
    private double targetPitchDegrees = 0.0;
    private double targetYawDegrees = 0.0;
    private boolean hasValidShot = false;
    private ShotCandidate lastShot = null;
    private TrajectoryResult lastTrajectoryResult = null;
    private Pose3d[] lastFlightPath = new Pose3d[0];
    
    private Supplier<Pose2d> poseSupplier = null;
    
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier = null;
    
    private double shooterHeightMeters = 0.6;
    private Translation2d shooterOffset = new Translation2d(0.2, 0);
    // 7.5 - 43.5
    private double minPitchDegrees = 0;
    private double maxPitchDegrees = 90;
    private double minArcHeightMeters = 0.75;
    private double targetRadiusMeters = 0.53;
    private Translation3d targetPosition = new Translation3d(6.0, 4.0, 5.1);

    private ObstacleConfig hubObstacle;
    private static final double FIELD_LENGTH_METERS = 16.46;
    private Pose3d[] cachedHubObstaclePoints = new Pose3d[0];
    private boolean hubObstaclePointsCacheValid = false;

    private boolean collisionEnabled = true;
    private double preferredArcHeightMeters = 2.5;
    private double arcBiasStrength = 0.6;
    private boolean isRedAlliance = false;
    private boolean trackingEnabled = true;
    private boolean debugExtendTrajectory = true;
    private boolean stationarySolveEnabled = true;
    private static final double STATIONARY_ANGLE_STEP = 0.01;
    private static final int STATIONARY_MAX_CANDIDATES = 1000;

    private Supplier<Double> currentRpmSupplier = null;
    private boolean rpmFeedbackEnabled = false;
    private double rpmFeedbackThreshold = 50.0;
    private double idealTargetRpm = 0.0;
    private double feedbackAdjustedPitchDegrees = Double.NaN;

    private double lastDistanceToTarget = 0.0;
    private double lastMaxTrajectoryHeight = 0.0;
    private boolean lastShotClearsHub = false;
    private String lastCollisionWarning = "";

    public ExampleShooter() {
        super();

        flywheelLeader = new MotorWrapper(MotorType.TALONFX, 40);
   

        SolverConstants.setHoopToleranceMultiplier(10000.0);
        SolverConstants.setBasketDescentToleranceMultiplier(6.0);
        SolverConstants.setMinTargetDistanceMeters(0.05);
        SolverConstants.setVelocityBufferMultiplier(1.2);

        gamePiece = GamePieces.REBUILT_2026_BALL;

        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.defaults()
                .toBuilder()
                .hoopToleranceMultiplier(10.0)
                .build();
        solver = new TrajectorySolver(gamePiece, solverConfig);
        solver.setDebugEnabled(true);

        hubObstacle = ObstacleConfig.builder()
                .position(4.03, 4.0)
                .baseSize(1.19, 1.19)
                .wallHeight(1.83)
                .upperStructureHeight(0.41)
                .openingDiameter(1.06)
                .enabled(true)
                .build();

        FlywheelGenerator generator = new FlywheelGenerator(gamePiece);
        FlywheelGenerator.GenerationResult result = generator.generateAndEvaluate(15.0);

        if (result.bestConfig != null) {
            flywheelConfig = result.bestConfig.config;
            System.out.println("Using flywheel config: " + flywheelConfig.getName());
        } else {
            flywheelConfig = FlywheelConfig.builder()
                    .name("2026 Shooter")
                    .arrangement(FlywheelConfig.WheelArrangement.DUAL_PARALLEL)
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
     * Set the chassis speeds supplier for FuelSim shooting.
     * This allows the shooter to account for robot velocity when launching.
     * 
     * @param supplier A supplier that returns field-relative ChassisSpeeds
     */
    public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> supplier) {
        this.chassisSpeedsSupplier = supplier;
    }
    
    /**
     * Set the shooter offset from robot center (in robot frame).
     * 
     * @param offset Translation2d representing shooter position relative to robot center
     */
    public void setShooterOffset(Translation2d offset) {
        this.shooterOffset = offset;
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
     * Set the target radius (goal opening size).
     * Larger values make hit detection more forgiving.
     * Standard basketball hoop is ~0.457m (18 inches).
     * 
     * @param radiusMeters Target radius in meters
     */
    public void setTargetRadius(double radiusMeters) {
        this.targetRadiusMeters = radiusMeters;
    }
    
    /**
     * Set the hub center position on the field.
     * Rebuilds the obstacle with the new position.
     * 
     * @param x Hub center X position in meters
     * @param y Hub center Y position in meters
     */
    public void setHubPosition(double x, double y) {
        hubObstacle = ObstacleConfig.builder()
                .position(x, y)
                .baseSize(hubObstacle.getBaseSizeX(), hubObstacle.getBaseSizeY())
                .wallHeight(hubObstacle.getWallHeight())
                .upperStructureHeight(hubObstacle.getUpperStructureHeight())
                .openingDiameter(hubObstacle.getOpeningDiameter())
                .collisionMargin(hubObstacle.getCollisionMargin())
                .enabled(hubObstacle.isEnabled())
                .build();
        hubObstaclePointsCacheValid = false;
    }
    
    /**
     * Set whether this is red alliance (mirrors obstacle positions).
     * When red alliance, the hub obstacle X position is mirrored across the field center.
     * 
     * @param redAlliance true for red alliance, false for blue
     */
    public void setAlliance(boolean redAlliance) {
        this.isRedAlliance = redAlliance;
        hubObstaclePointsCacheValid = false;
    }
    
    /**
     * Enable or disable collision-aware trajectory solving.
     * When enabled, the solver will reject trajectories that pass through the hub.
     * 
     * @param enabled true to enable collision checking
     */
    public void setCollisionEnabled(boolean enabled) {
        this.collisionEnabled = enabled;
    }
    
    /**
     * Set the hub obstacle enabled state directly.
     * 
     * @param enabled true to enable the hub obstacle for collision detection
     */
    public void setHubObstacleEnabled(boolean enabled) {
        hubObstacle = ObstacleConfig.builder()
                .position(hubObstacle.getCenterX(), hubObstacle.getCenterY())
                .baseSize(hubObstacle.getBaseSizeX(), hubObstacle.getBaseSizeY())
                .wallHeight(hubObstacle.getWallHeight())
                .upperStructureHeight(hubObstacle.getUpperStructureHeight())
                .openingDiameter(hubObstacle.getOpeningDiameter())
                .collisionMargin(hubObstacle.getCollisionMargin())
                .enabled(enabled)
                .build();
        hubObstaclePointsCacheValid = false;
    }
    
    /**
     * Set the preferred arc height for shots. Higher values encourage steeper arcs.
     * 
     * @param heightMeters Preferred arc height in meters (e.g., 2.5 for hub clearance)
     */
    public void setPreferredArcHeight(double heightMeters) {
        this.preferredArcHeightMeters = Math.max(0.0, heightMeters);
    }
    
    /**
     * Set the arc bias strength. Controls how strongly the solver prefers
     * trajectories matching the preferred arc height.
     * 
     * @param strength 0.0 = no bias, 1.0 = strongly prefer preferred arc height
     */
    public void setArcBiasStrength(double strength) {
        this.arcBiasStrength = Math.max(0.0, Math.min(1.0, strength));
    }
    
    /**
     * Replace the hub obstacle with a custom obstacle configuration.
     * 
     * @param obstacle The new obstacle configuration
     */
    public void setHubObstacle(ObstacleConfig obstacle) {
        this.hubObstacle = obstacle;
        hubObstaclePointsCacheValid = false;
    }
    
    /**
     * Get the current hub obstacle configuration.
     * 
     * @return The current ObstacleConfig for the hub
     */
    public ObstacleConfig getHubObstacle() {
        return hubObstacle;
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
     * Set the minimum arc height above the target.
     * This ensures the ball reaches a certain height before descending into the goal.
     * Higher values create more pronounced arcs that drop into the target.
     * 
     * @param heightMeters Minimum height above target that the trajectory apex must reach (0 = no minimum)
     */
    public void setMinArcHeight(double heightMeters) {
        this.minArcHeightMeters = Math.max(0.0, heightMeters);
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

    /**
     * Enable or disable debug trajectory extension.
     * When enabled, the trajectory visualization extends past the target
     * until the ball hits the ground, showing the full flight path.
     * Useful for verifying arc shapes in AdvantageScope.
     * 
     * @param enabled true to show the full extended trajectory
     */
    public void setDebugExtendTrajectory(boolean enabled) {
        this.debugExtendTrajectory = enabled;
    }

    /**
     * Check if debug trajectory extension is enabled.
     */
    public boolean isDebugExtendTrajectory() {
        return debugExtendTrajectory;
    }

    /**
     * Enable or disable stationary (aggressive) solve mode.
     * When enabled, the solver uses a much finer angle step and evaluates
     * far more candidates to find the absolute best trajectory.
     * Use this when the robot is standing still and accuracy matters more than speed.
     * 
     * @param enabled true to enable aggressive solving
     */
    public void setStationarySolveEnabled(boolean enabled) {
        this.stationarySolveEnabled = enabled;
    }

    /**
     * Check if stationary solve mode is enabled.
     */
    public boolean isStationarySolveEnabled() {
        return stationarySolveEnabled;
    }

    /**
     * Sets the current RPM supplier for flywheel feedback.
     * This should read the actual flywheel RPM from an encoder or sensor.
     * 
     * @param supplier Supplier that returns the current flywheel RPM
     */
    public void setCurrentRpmSupplier(Supplier<Double> supplier) {
        this.currentRpmSupplier = supplier;
    }

    /**
     * Enable or disable RPM feedback mode.
     * When enabled, the solver adjusts the pitch angle based on the actual flywheel RPM
     * instead of assuming the flywheel has reached the target RPM. This compensates for
     * flywheel spindown during rapid-fire shooting.
     * 
     * @param enabled true to enable RPM feedback
     */
    public void setRpmFeedbackEnabled(boolean enabled) {
        this.rpmFeedbackEnabled = enabled;
    }

    /**
     * Check if RPM feedback mode is enabled.
     */
    public boolean isRpmFeedbackEnabled() {
        return rpmFeedbackEnabled;
    }

    /**
     * Sets the RPM threshold below which feedback re-solving kicks in.
     * If the actual RPM is more than this amount below the target RPM,
     * the solver will find a new angle for the current velocity.
     * 
     * @param threshold RPM difference threshold (default 50.0)
     */
    public void setRpmFeedbackThreshold(double threshold) {
        this.rpmFeedbackThreshold = threshold;
    }

    @Override
    public void periodic() {
        if (trackingEnabled && poseSupplier != null) {
            Pose2d currentPose = poseSupplier.get();
            calculateShot(
                currentPose.getX(), currentPose.getY(), shooterHeightMeters,
                targetPosition.getX(), targetPosition.getY(), targetPosition.getZ()
            );
            recordOutput("RobotX", currentPose.getX());
            recordOutput("RobotY", currentPose.getY());
        }
        
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
        
        recordOutput("MaxTrajectoryHeight", lastMaxTrajectoryHeight);
        recordOutput("ShotClearsHub", lastShotClearsHub);
        recordOutput("CollisionEnabled", collisionEnabled);
        recordOutput("ArcBiasStrength", arcBiasStrength);
        recordOutput("PreferredArcHeight", preferredArcHeightMeters);
        recordOutput("IsRedAlliance", isRedAlliance);
        recordOutput("DebugExtendTrajectory", debugExtendTrajectory);
        recordOutput("StationarySolveEnabled", stationarySolveEnabled);

        recordOutput("RpmFeedbackEnabled", rpmFeedbackEnabled);
        recordOutput("IdealTargetRPM", idealTargetRpm);
        if (currentRpmSupplier != null) {
            recordOutput("MeasuredRPM", currentRpmSupplier.get());
            recordOutput("RpmDeficit", idealTargetRpm - currentRpmSupplier.get());
        }
        if (!Double.isNaN(feedbackAdjustedPitchDegrees)) {
            recordOutput("FeedbackAdjustedPitch", feedbackAdjustedPitchDegrees);
        }

        ObstacleConfig effectiveObs = isRedAlliance ? hubObstacle.mirrorX(FIELD_LENGTH_METERS) : hubObstacle;
        recordOutput("HubObstacleEnabled", hubObstacle.isEnabled());
        recordOutput("HubTotalHeight", effectiveObs.getTotalHeight());
        recordOutput("HubCenterX", effectiveObs.getCenterX());
        recordOutput("HubCenterY", effectiveObs.getCenterY());
        if (!lastCollisionWarning.isEmpty()) {
            recordOutput("CollisionWarning", lastCollisionWarning);
        } else {
            recordOutput("CollisionWarning", "None");
        }
        // Calculate and log clearance metrics
        double heightClearance = lastMaxTrajectoryHeight - effectiveObs.getTotalHeight();
        recordOutput("HeightClearance", heightClearance);
        recordOutput("TrajectoryArcOK", heightClearance > 0);
        
        if (!hubObstaclePointsCacheValid) {
            cachedHubObstaclePoints = generateHubObstaclePoints(effectiveObs);
            hubObstaclePointsCacheValid = true;
        }
        Logger.recordOutput("ExampleShooter/HubObstacle", cachedHubObstaclePoints);

        // ===== Debug info: rejection stats + candidate trajectories =====
        publishDebugInfo();
    }

    /**
     * Publishes solver debug information to NetworkTables.
     * Shows rejection breakdown and all candidate trajectory paths for visualization.
     */
    private void publishDebugInfo() {
        if (lastTrajectoryResult == null) return;
        SolveDebugInfo debug = lastTrajectoryResult.getDebugInfo();
        if (debug == null) {
            recordOutput("Debug/Enabled", false);
            return;
        }
        recordOutput("Debug/Enabled", true);
        recordOutput("Debug/TotalTested", debug.getTotalTested());
        recordOutput("Debug/Accepted", debug.getAcceptedCount());
        recordOutput("Debug/TotalRejected", debug.getTotalRejected());
        recordOutput("Debug/RejectedCollision", debug.getRejectedCollisionCount());
        recordOutput("Debug/RejectedArcTooLow", debug.getRejectedArcTooLowCount());
        recordOutput("Debug/RejectedClearance", debug.getRejectedClearanceCount());
        recordOutput("Debug/RejectedMiss", debug.getRejectedMissCount());
        recordOutput("Debug/RejectedFlyover", debug.getRejectedFlyoverCount());
        recordOutput("Debug/BestScore", debug.getBestScore());
        recordOutput("Debug/BestPitchDeg", debug.getBestPitchDegrees());
        recordOutput("Debug/Summary", debug.getSummary());

        // Publish each accepted candidate's trajectory as a separate Pose3d[]
        java.util.List<SolveDebugInfo.CandidateInfo> accepted = debug.getAcceptedCandidates();
        recordOutput("Debug/AcceptedPaths", accepted.size());

        // Publish all accepted paths combined into numbered keys
        int pathIndex = 0;
        for (SolveDebugInfo.CandidateInfo candidate : accepted) {
            if (candidate.getTrajectory() == null) continue;
            Pose3d[] path = trajectoryStatesToPose3d(candidate.getTrajectory());
            if (path.length > 0) {
                Logger.recordOutput("ExampleShooter/CandidatePath" + pathIndex, path);
                recordOutput("Debug/Candidate" + pathIndex + "/Pitch", candidate.getPitchDegrees());
                recordOutput("Debug/Candidate" + pathIndex + "/Score", candidate.getScore());
                recordOutput("Debug/Candidate" + pathIndex + "/Closest", candidate.getClosestApproach());
                recordOutput("Debug/Candidate" + pathIndex + "/TOF", candidate.getTimeOfFlight());
                recordOutput("Debug/Candidate" + pathIndex + "/MaxHeight", candidate.getMaxHeight());
                pathIndex++;
            }
            if (pathIndex >= 10) break; // Cap at 10 paths to limit NT bandwidth
        }
        // Clear any stale paths beyond current count
        for (int i = pathIndex; i < 10; i++) {
            Logger.recordOutput("ExampleShooter/CandidatePath" + i, new Pose3d[0]);
        }

        // Publish a few rejected paths (sample: first of each rejection type)
        int rejIndex = 0;
        boolean[] typeSeen = new boolean[SolveDebugInfo.RejectionReason.values().length];
        for (SolveDebugInfo.CandidateInfo candidate : debug.getRejectedCandidates()) {
            int ord = candidate.getRejection().ordinal();
            if (typeSeen[ord]) continue;
            typeSeen[ord] = true;
            if (candidate.getTrajectory() != null) {
                Pose3d[] path = trajectoryStatesToPose3d(candidate.getTrajectory());
                if (path.length > 0) {
                    Logger.recordOutput("ExampleShooter/RejectedPath_" + candidate.getRejection().name(), path);
                    recordOutput("Debug/Rejected_" + candidate.getRejection().name() + "/Pitch", candidate.getPitchDegrees());
                    rejIndex++;
                }
            }
            if (rejIndex >= 5) break;
        }
    }

    /**
     * Converts raw TrajectoryState array to Pose3d array for AdvantageScope visualization.
     */
    private static Pose3d[] trajectoryStatesToPose3d(
            ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion.TrajectoryState[] states) {
        if (states == null) return new Pose3d[0];
        List<Pose3d> poses = new ArrayList<>();
        for (var state : states) {
            if (state == null) continue;
            Translation3d pos = new Translation3d(state.x, state.y, state.z);
            Rotation3d rot = new Rotation3d(
                    0.0,
                    Math.atan2(state.vz, Math.sqrt(state.vx * state.vx + state.vy * state.vy)),
                    Math.atan2(state.vy, state.vx)
            );
            poses.add(new Pose3d(pos, rot));
        }
        return poses.toArray(new Pose3d[0]);
    }

    private Pose3d[] generateHubObstaclePoints(ObstacleConfig obstacle) {
        List<Pose3d> points = new ArrayList<>();
        double centerX = obstacle.getCenterX();
        double centerY = obstacle.getCenterY();
        double sizeX = obstacle.getBaseSizeX();
        double sizeY = obstacle.getBaseSizeY();
        double totalHeight = obstacle.getTotalHeight();
        
        double halfX = sizeX / 2.0;
        double halfY = sizeY / 2.0;
        
        for (double x = centerX - halfX; x <= centerX + halfX; x += 0.2) {
            for (double y = centerY - halfY; y <= centerY + halfY; y += 0.2) {
                for (double z = 0; z <= totalHeight; z += 0.4) {
                    points.add(new Pose3d(new Translation3d(x, y, z), new Rotation3d()));
                }
            }
        }
        
        return points.toArray(new Pose3d[0]);
    }

    /**
     * Calculate shot parameters for a target position. Call this before
     * shooting to update targetRpm and targetPitchDegrees.
     * 
     * Now includes robot velocity compensation and proper target radius.
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
        
        double robotVx = 0.0;
        double robotVy = 0.0;
        if (chassisSpeedsSupplier != null) {
            ChassisSpeeds speeds = chassisSpeedsSupplier.get();
            robotVx = speeds.vxMetersPerSecond;
            robotVy = speeds.vyMetersPerSecond;
        }
        
        ObstacleConfig effectiveObstacle = isRedAlliance
                ? hubObstacle.mirrorX(FIELD_LENGTH_METERS)
                : hubObstacle;

        ShotInput.Builder inputBuilder = ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterZ)
                .shooterYawRadians(yawToTargetRadians)
                .targetPositionMeters(targetX, targetY, targetZ)
                .targetRadiusMeters(targetRadiusMeters)
                .robotVelocity(robotVx, robotVy)
                .pitchRangeDegrees(minPitchDegrees, maxPitchDegrees)
                .minArcHeightMeters(minArcHeightMeters)
                .shotPreference(ShotInput.ShotPreference.AUTO)
                .addObstacle(effectiveObstacle)
                .collisionCheckEnabled(collisionEnabled)
                .preferredArcHeightMeters(preferredArcHeightMeters)
                .arcBiasStrength(arcBiasStrength);
        
        if (stationarySolveEnabled) {
            inputBuilder.angleStepDegrees(STATIONARY_ANGLE_STEP)
                    .maxCandidates(STATIONARY_MAX_CANDIDATES);
        }
        
        ShotInput input = inputBuilder.build();

        lastTrajectoryResult = solver.solve(input);
        
        if (lastTrajectoryResult.isSuccess()) {
            List<Pose3d> flightPathList = lastTrajectoryResult.getFlightPath();
            
            if (debugExtendTrajectory) {
                ProjectileMotion debugSim = new ProjectileMotion();
                ProjectileMotion.TrajectoryResult debugResult = debugSim.simulate(
                        gamePiece,
                        shooterX, shooterY, shooterZ,
                        lastTrajectoryResult.getRequiredVelocityMps(),
                        lastTrajectoryResult.getPitchAngleRadians(),
                        yawToTargetRadians,
                        0,
                        targetX, targetY, 1000.0,
                        targetRadiusMeters
                );
                List<Pose3d> debugPath = new ArrayList<>();
                for (ProjectileMotion.TrajectoryState state : debugResult.trajectory) {
                    if (state == null) continue;
                    Translation3d pos = new Translation3d(state.x, state.y, state.z);
                    Rotation3d rot = new Rotation3d(
                            0.0,
                            Math.atan2(state.vz, Math.sqrt(state.vx * state.vx + state.vy * state.vy)),
                            Math.atan2(state.vy, state.vx)
                    );
                    debugPath.add(new Pose3d(pos, rot));
                }
                lastFlightPath = debugPath.toArray(new Pose3d[0]);
            } else {
                lastFlightPath = flightPathList.toArray(new Pose3d[0]);
            }
            
            lastMaxTrajectoryHeight = lastTrajectoryResult.getMaxHeightMeters();
            lastShotClearsHub = lastMaxTrajectoryHeight > effectiveObstacle.getTotalHeight();
            lastCollisionWarning = "";

            double clearance = lastMaxTrajectoryHeight - effectiveObstacle.getTotalHeight();
            if (clearance < 0.1 && clearance > 0) {
                lastCollisionWarning = String.format("LOW CLEARANCE: %.2fm above hub structure", clearance);
            }
            
            targetPitchDegrees = lastTrajectoryResult.getPitchAngleDegrees();
            targetYawDegrees = Math.toDegrees(yawToTargetRadians);
            targetRpm = lastTrajectoryResult.getRecommendedRpm();
            idealTargetRpm = targetRpm;
            feedbackAdjustedPitchDegrees = Double.NaN;
            hasValidShot = true;
            
            if (rpmFeedbackEnabled && currentRpmSupplier != null) {
                double measuredRpm = currentRpmSupplier.get();
                double rpmDeficit = idealTargetRpm - measuredRpm;
                
                if (rpmDeficit > rpmFeedbackThreshold && measuredRpm > 0) {
                    TrajectoryResult feedbackResult = solver.solveAtCurrentRpm(
                        input, flywheelConfig, measuredRpm
                    );
                    
                    if (feedbackResult.isSuccess()) {
                        feedbackAdjustedPitchDegrees = feedbackResult.getPitchAngleDegrees();
                        targetPitchDegrees = feedbackAdjustedPitchDegrees;
                        lastTrajectoryResult = feedbackResult;
                    }
                }
            }
            
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
            Optional<ShotCandidate> fastest = candidates.getBest();
            
            if (fastest.isPresent()) {
                lastShot = fastest.get();
                targetPitchDegrees = lastShot.getPitchAngleDegrees();
                targetYawDegrees = lastShot.getYawAngleDegrees();
                targetRpm = lastShot.getRequiredVelocityMps() * 300.0;
                hasValidShot = true;
                lastFlightPath = new Pose3d[0];
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
    
    /**
     * Shoots a ball in simulation using FuelSim.
     * Uses the current robot pose, chassis speeds, and calculated trajectory parameters.
     * The ball is spawned with velocity based on the target RPM and pitch angle.
     */
    public void shootBallSim() {
        if (poseSupplier == null) {
            System.out.println("Cannot shoot: pose supplier not set");
            return;
        }
        
        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier != null ? 
                chassisSpeedsSupplier.get() : new ChassisSpeeds();
        
        double launchSpeed;
        double launchAngleRad;
        double yawRad;
        
        if (hasValidShot && lastTrajectoryResult != null && lastTrajectoryResult.isSuccess()) {
            launchSpeed = lastTrajectoryResult.getRequiredVelocityMps();
            launchAngleRad = lastTrajectoryResult.getPitchAngleRadians();
            yawRad = Math.toRadians(targetYawDegrees);
        } else if (hasValidShot && lastShot != null) {
            launchSpeed = lastShot.getRequiredVelocityMps();
            launchAngleRad = lastShot.getPitchAngleRadians();
            yawRad = lastShot.getYawAngleRadians();
        } else {
            launchSpeed = 15.0;
            launchAngleRad = Math.toRadians(55);
            yawRad = robotPose.getRotation().getRadians();
        }

        double cosYaw = Math.cos(yawRad);
        double sinYaw = Math.sin(yawRad);
        
        Rotation2d shooterRotation = robotPose.getRotation();
        double cosTheta = shooterRotation.getCos();
        double sinTheta = shooterRotation.getSin();
        double worldOffsetX = shooterOffset.getX() * cosTheta - shooterOffset.getY() * sinTheta;
        double worldOffsetY = shooterOffset.getX() * sinTheta + shooterOffset.getY() * cosTheta;
        
        Translation3d shooterPos = new Translation3d(
                robotPose.getX() + worldOffsetX,
                robotPose.getY() + worldOffsetY,
                shooterHeightMeters
        );
        
        double horizontalSpeed = launchSpeed * Math.cos(launchAngleRad);
        double verticalSpeed = launchSpeed * Math.sin(launchAngleRad);

        Translation3d launchVel = new Translation3d(
                horizontalSpeed * cosYaw + chassisSpeeds.vxMetersPerSecond,
                horizontalSpeed * sinYaw + chassisSpeeds.vyMetersPerSecond,
                verticalSpeed
        );
        
        FuelSim.getInstance().spawnFuel(shooterPos, launchVel);
        
        System.out.println("Shot ball! Pos: " + shooterPos + " Vel: " + launchVel + 
                " Speed: " + launchSpeed + " m/s, Angle: " + Math.toDegrees(launchAngleRad) + " deg");
    }
    
    /**
     * Command to shoot a ball in simulation.
     * @return Command that shoots a single ball
     */
    public Command shootBallSimCommand() {
        return runOnce(this::shootBallSim);
    }

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
                builder.addDoubleProperty("TimeOfFlight", () -> lastShot != null ? lastShot.getTimeOfFlightSeconds() : 0.0, null);
                builder.addDoubleProperty("MaxHeight", () -> lastShot != null ? lastShot.getMaxHeightMeters() : 0.0, null);
                builder.addDoubleProperty("RequiredVelocity", () -> lastShot != null ? lastShot.getRequiredVelocityMps() : 0.0, null);
                builder.addDoubleProperty("Confidence", () -> lastShot != null ? lastShot.getAccuracyScore() : 0.0, null);
                builder.addDoubleProperty("MarginOfError", () -> lastTrajectoryResult != null && lastTrajectoryResult.isSuccess() 
                        ? lastTrajectoryResult.getMarginOfErrorMeters() : 0.0, null);
            }
        };
    }
}
