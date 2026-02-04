package frc.robot.subsystems;

import ca.team4308.absolutelib.vision.Vision;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.SwerveDrive;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Example vision subsystem demonstrating PhotonVision integration. Uses the
 * Vision class for AprilTag-based pose estimation.
 */
public class ExampleVision extends AbsoluteSubsystem {

    private final Vision vision;
    private final Supplier<Pose2d> poseSupplier;

    // Vision state
    private boolean hasTarget = false;
    private double lastPoseX = 0.0;
    private double lastPoseY = 0.0;
    private double lastPoseRotDeg = 0.0;

    public ExampleVision(SwerveDrive swerveDrive) {
        super();

        this.poseSupplier = swerveDrive::getPose;

        // Load field layout for 2025 Reefscape (or current year)
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        // Configure front camera
        // Camera mounted 0.3m forward, 0.2m up, facing forward
        Vision.VisionCamera frontCamera = new Vision.VisionCamera(
                "FrontCamera", // Camera name in PhotonVision
                fieldLayout,
                new Rotation3d(0, 0, 0), // No rotation
                new Translation3d(0.30, 0.0, 0.20), // 30cm forward, 20cm up
                VecBuilder.fill(0.8, 0.8, 2.0), // Single-tag std devs
                VecBuilder.fill(0.3, 0.3, 1.0) // Multi-tag std devs
        );

        // Create vision system
        vision = new Vision(
                poseSupplier,
                swerveDrive.field,
                fieldLayout,
                frontCamera
        );
    }

    /**
     * Constructor for when not using swerve drive directly. Requires a pose
     * supplier and field2d instance.
     */
    public ExampleVision(Supplier<Pose2d> poseSupplier, Field2d field) {
        super();

        this.poseSupplier = poseSupplier;

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        Vision.VisionCamera frontCamera = new Vision.VisionCamera(
                "FrontCamera",
                fieldLayout,
                new Rotation3d(0, 0, 0),
                new Translation3d(0.30, 0.0, 0.20),
                VecBuilder.fill(0.8, 0.8, 2.0),
                VecBuilder.fill(0.3, 0.3, 1.0)
        );

        vision = new Vision(poseSupplier, field, fieldLayout, frontCamera);
    }

    @Override
    public void periodic() {
        // Update vision tracking on the field display
        vision.updateVisionField();

        // Update state for logging
        Pose2d currentPose = poseSupplier.get();
        lastPoseX = currentPose.getX();
        lastPoseY = currentPose.getY();
        lastPoseRotDeg = currentPose.getRotation().getDegrees();
    }

    /**
     * Update pose estimation and feed to swerve drive odometry. Call this in
     * teleopPeriodic/autoPeriodic.
     */
    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        vision.updatePoseEstimation(swerveDrive);
    }

    /**
     * Get the pose to a specific AprilTag with an offset. Useful for aligning
     * to scoring positions.
     *
     * @param tagId The AprilTag ID to target
     * @param offset Transform to apply from the tag pose
     * @return Target pose, or empty if tag not in layout
     */
    public Optional<Pose2d> getTargetPose(int tagId, Transform2d offset) {
        return Optional.ofNullable(vision.getAprilTagPose(tagId, offset));
    }

    /**
     * Get distance from robot to a specific AprilTag.
     *
     * @param tagId The AprilTag ID
     * @return Distance in meters, or -1 if not visible
     */
    public double getDistanceToTag(int tagId) {
        return vision.getDistanceFromAprilTag(tagId);
    }

    /**
     * Check if vision has detected any targets recently.
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    @Override
    public Sendable log() {
        return builder -> {
            builder.setSmartDashboardType("ExampleVision");
            builder.addBooleanProperty("HasTarget", () -> hasTarget, null);
            builder.addDoubleProperty("PoseX", () -> lastPoseX, null);
            builder.addDoubleProperty("PoseY", () -> lastPoseY, null);
            builder.addDoubleProperty("PoseRotDeg", () -> lastPoseRotDeg, null);
        };
    }
}
