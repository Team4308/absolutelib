package ca.team4308.absolutelib.vision;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * A generic Vision class for PhotonVision integration with YAGSL.
 */
public class Vision {

    private final AprilTagFieldLayout fieldLayout;
    // Ambiguity defined as a value between (0,1). Could be used in filtering poses
    public static final double maximumAmbiguity = 0.20;

    public VisionSystemSim visionSim;
    private final Supplier<Pose2d> currentPose;
    private final Field2d field2d;
    private final List<VisionCamera> cameras = new ArrayList<>();

    // Object detection
    private PhotonCamera objCamera;
    private double objectOffset = 0.0;
    public boolean hasValidObject = false;

    /**
     * Constructor for the Vision class.
     *
     * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
     * @param field       Current field, should be {@link SwerveDrive#field}
     * @param fieldLayout The AprilTag field layout to use
     * @param cameras     Varargs of VisionCamera configurations
     */
    public Vision(Supplier<Pose2d> currentPose, Field2d field, AprilTagFieldLayout fieldLayout, VisionCamera... cameras) {
        this.currentPose = currentPose;
        this.field2d = field;
        this.fieldLayout = fieldLayout;
        this.cameras.addAll(Arrays.asList(cameras));

        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim("Vision");
            visionSim.addAprilTags(fieldLayout);

            for (VisionCamera c : this.cameras) {
                c.addToVisionSim(visionSim);
            }
        }
    }

    /**
     * Initialize object detection camera.
     * @param cameraName Name of the camera for object detection
     */
    public void configObjectDetection(String cameraName) {
        this.objCamera = new PhotonCamera(cameraName);
    }

    /**
     * Calculates a target pose relative to an AprilTag on the field.
     *
     * @param aprilTag    The ID of the AprilTag.
     * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose.
     * @return The target pose of the AprilTag.
     */
    public Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
        Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
        }
    }

    /**
     * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
     *
     * @param swerveDrive {@link SwerveDrive} instance.
     */
    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
            visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
        }
        for (VisionCamera camera : cameras) {
            Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
            if (poseEst.isPresent()) {
                var pose = poseEst.get();
                swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                        pose.timestampSeconds,
                        camera.curStdDevs);
            }
        }
    }

    public Supplier<OptionalDouble> getObjectOffset() {
        return () -> OptionalDouble.of(objectOffset);
    }

    public void updateObjectOffset() {
        if (objCamera == null) return;

        var results = objCamera.getLatestResult().getTargets();
        boolean changed = false;

        if (!results.isEmpty()) {
            for (int i = 0; i < results.size(); i++) {
                PhotonTrackedTarget result = results.get(i);
                if (result.getDetectedObjectClassID() == 1) {
                    objectOffset = result.getYaw();
                    hasValidObject = true;
                    changed = true;
                    break;
                }
            }
        }
        if (!changed) {
            objectOffset = 0.0;
            hasValidObject = false;
        }
        Logger.recordOutput("Vision/ObjectOffset", objectOffset);
    }

    /**
     * Generates the estimated robot pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(VisionCamera camera) {
        Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
        if (RobotBase.isSimulation()) {
            Field2d debugField = visionSim.getDebugField();
            poseEst.ifPresentOrElse(
                    est -> debugField
                            .getObject("VisionEstimation_" + camera.camera.getName())
                            .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        debugField.getObject("VisionEstimation_" + camera.camera.getName()).setPoses();
                    });
        }
        return poseEst;
    }

    /**
     * Get distance of the robot from the AprilTag pose.
     */
    public double getDistanceFromAprilTag(int id) {
        Optional<Pose3d> tag = fieldLayout.getTagPose(id);
        return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
    }

    /**
     * Vision simulation.
     */
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }

    /**
     * Update the {@link Field2d} to include tracked targets
     */
    public void updateVisionField() {
        List<Pose2d> poses = new ArrayList<>();
        for (VisionCamera c : cameras) {
            if (!c.resultsList.isEmpty()) {
                PhotonPipelineResult latest = c.resultsList.get(0);
                if (latest.hasTargets()) {
                    for (PhotonTrackedTarget target : latest.targets) {
                        if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                            Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
                            poses.add(targetPose);
                        }
                    }
                }
            }
        }
        Logger.recordOutput("Vision/Targets", poses.toArray(new Pose2d[0]));
        field2d.getObject("Vision Targets").setPoses(poses);
    }

    /**
     * Class representing a camera configuration and logic.
     */
    public static class VisionCamera {
        public final Alert latencyAlert;
        public final PhotonCamera camera;
        public final PhotonPoseEstimator poseEstimator;
        private final Matrix<N3, N1> singleTagStdDevs;
        private final Matrix<N3, N1> multiTagStdDevs;
        private final Transform3d robotToCamTransform;
        public Matrix<N3, N1> curStdDevs;
        public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        public PhotonCameraSim cameraSim;
        public List<PhotonPipelineResult> resultsList = new ArrayList<>();
        private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

        public VisionCamera(String name, AprilTagFieldLayout layout, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
                            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs) {
            latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
            camera = new PhotonCamera(name);
            robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
            poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTransform);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevs;

            if (RobotBase.isSimulation()) {
                SimCameraProperties cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(960, 800, Rotation2d.fromDegrees(100));
                cameraProp.setCalibError(0.25, 0.08);
                cameraProp.setFPS(30);
                cameraProp.setAvgLatencyMs(35);
                cameraProp.setLatencyStdDevMs(5);
                cameraSim = new PhotonCameraSim(camera, cameraProp);
                cameraSim.enableDrawWireframe(true);
            }
        }

        public void addToVisionSim(VisionSystemSim systemSim) {
            if (RobotBase.isSimulation()) {
                systemSim.addCamera(cameraSim, robotToCamTransform);
            }
        }

        public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
            updateUnreadResults();
            return estimatedRobotPose;
        }

        private void updateUnreadResults() {
            resultsList = RobotBase.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
            filter();
            resultsList.sort((a, b) -> a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1);
            if (!resultsList.isEmpty()) {
                updateEstimatedGlobalPose();
            }
        }

        private void updateEstimatedGlobalPose() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var change : resultsList) {
                visionEst = poseEstimator.update(change);
                updateEstimationStdDevs(visionEst, change.getTargets());
            }
            estimatedRobotPose = visionEst;
        }

        private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
            if (estimatedPose.isEmpty()) {
                curStdDevs = singleTagStdDevs;
            } else {
                var estStdDevs = singleTagStdDevs;
                int numTags = 0;
                double avgDist = 0;
                for (var tgt : targets) {
                    var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if (tagPose.isEmpty()) continue;
                    numTags++;
                    avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }
                if (numTags == 0) {
                    curStdDevs = singleTagStdDevs;
                } else {
                    avgDist /= numTags;
                    if (numTags > 1) estStdDevs = multiTagStdDevs;
                    if (numTags == 1 && avgDist > 4) {
                        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    } else {
                        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                    }
                    curStdDevs = estStdDevs;
                }
            }
        }

        private void filter() {
            resultsList.removeIf(result -> {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getPoseAmbiguity() > Vision.maximumAmbiguity) return true;
                }
                return false;
            });
        }
    }
}
