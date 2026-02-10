package frc.robot.subsystems;

import ca.team4308.absolutelib.math.trajectories.shooter.*;
import ca.team4308.absolutelib.math.trajectories.*;
import ca.team4308.absolutelib.math.trajectories.gamepiece.*;
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

import java.util.List;
import java.util.function.Supplier;


public class ExampleShooter extends AbsoluteSubsystem {

    private final MotorWrapper flywheelLeader;

    private final ShooterSystem shooterSystem;
    private final TrajectorySolver solver;

    private ShotParameters currentShot = ShotParameters.invalid("Not yet calculated");
    private double targetYawDegrees = 0.0;
    private double lastDistanceMeters = 0.0;

    private Supplier<Pose2d> poseSupplier = null;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier = null;
    private Supplier<Double> currentRpmSupplier = null;

    private double shooterHeightMeters = 0.6;
    private Translation2d shooterOffset = new Translation2d(0.2, 0);
    private Translation3d targetPosition = new Translation3d(6.0, 4.0, 2.1);
    private boolean trackingEnabled = true;

    public ExampleShooter() {
        super();

        flywheelLeader = new MotorWrapper(MotorType.TALONFX, 40);

        ShooterConfig config = ShooterConfig.builder()
                .pitchLimits(47.5, 82.5)
                .rpmLimits(500, 6000)
                .rpmToVelocityFactor(0.00532) 
                .distanceLimits(0.5, 12.0)
                .rpmFeedbackThreshold(50.0)
                .rpmAbortThreshold(500.0)
                .pitchCorrectionPerRpmDeficit(0.005)
                .movingCompensationGain(1.0)
                .movingIterations(5)
                .safetyMaxExitVelocity(30.0)
                .build();

        ShotLookupTable table = new ShotLookupTable()
                .addEntry(1.0, 78.0, 1800)   
                .addEntry(1.5, 75.0, 2000)
                .addEntry(2.0, 72.0, 2300)
                .addEntry(2.5, 68.0, 2600)
                .addEntry(3.0, 65.0, 2900)
                .addEntry(3.5, 62.0, 3200)
                .addEntry(4.0, 59.0, 3500)
                .addEntry(5.0, 55.0, 3900)
                .addEntry(6.0, 52.0, 4300)
                .addEntry(7.0, 50.0, 4700)
                .addEntry(8.0, 48.0, 5100);

        GamePiece gamePiece = GamePieces.REBUILT_2026_BALL;
        SolverConstants.setHoopToleranceMultiplier(10.0);
        SolverConstants.setBasketDescentToleranceMultiplier(6.0);
        SolverConstants.setMinTargetDistanceMeters(0.05);
        SolverConstants.setVelocityBufferMultiplier(1.2);
        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.defaults()
                .toBuilder().hoopToleranceMultiplier(10.0).build();
        solver = new TrajectorySolver(gamePiece, solverConfig);

        shooterSystem = new ShooterSystem(config, table, solver);
        shooterSystem.setMode(ShotMode.SOLVER_ONLY);
        shooterSystem.setFallbackShot(60.0, 3000);

        solver.setDebugEnabled(true);
    }

    public void setPoseSupplier(Supplier<Pose2d> supplier) { this.poseSupplier = supplier; }
    public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> supplier) { this.chassisSpeedsSupplier = supplier; }
    public void setCurrentRpmSupplier(Supplier<Double> supplier) { this.currentRpmSupplier = supplier; }
    public void setShooterHeight(double meters) { this.shooterHeightMeters = meters; }
    public void setShooterOffset(Translation2d offset) { this.shooterOffset = offset; }
    public void setTarget(double x, double y, double z) { this.targetPosition = new Translation3d(x, y, z); }
    public void setTrackingEnabled(boolean enabled) { this.trackingEnabled = enabled; }
    public boolean isTrackingEnabled() { return trackingEnabled; }
    public void setPitchLimits(double min, double max) { }

    /** Change the shot mode at runtime (e.g. from dashboard or button). */
    public void setMode(ShotMode mode) { shooterSystem.setMode(mode); }
    public ShotMode getMode() { return shooterSystem.getMode(); }

    /** Set manual override values for MANUAL mode. */
    public void setManualOverride(double pitchDegrees, double rpm) {
        shooterSystem.setManualOverride(pitchDegrees, rpm);
    }


    @Override
    public void periodic() {
        if (trackingEnabled && poseSupplier != null) {
            updateShot();
        }

        if (currentShot.valid && currentShot.rpm > 0) {
            flywheelLeader.set(Math.min(currentShot.rpm / 6000.0, 1.0));
        }

        recordOutput("ValidShot", currentShot.valid);
        recordOutput("TargetRPM", currentShot.rpm);
        recordOutput("TargetPitchDeg", currentShot.pitchDegrees);
        recordOutput("TargetYawDeg", targetYawDegrees);
        recordOutput("Distance", lastDistanceMeters);
        recordOutput("ShotSource", currentShot.source.name());
        recordOutput("Mode", shooterSystem.getMode().name());
        recordOutput("SourceDetail", shooterSystem.getLastSourceDescription());
        recordOutput("TrackingEnabled", trackingEnabled);

        if (currentRpmSupplier != null) {
            double measured = currentRpmSupplier.get();
            recordOutput("MeasuredRPM", measured);
            recordOutput("RpmDeficit", currentShot.rpm - measured);
            recordOutput("ReadyToFire", shooterSystem.isReadyToFire(measured));
        }

        Pose3d goalPose = new Pose3d(targetPosition, new Rotation3d());
        Logger.recordOutput("ExampleShooter/GoalPose3d", goalPose);
        Logger.recordOutput("ExampleShooter/GoalPose3dArray", new Pose3d[] { goalPose });

        recordOutput("TargetX", targetPosition.getX());
        recordOutput("TargetY", targetPosition.getY());
        recordOutput("TargetZ", targetPosition.getZ());
        recordOutput("ShooterHeight", shooterHeightMeters);
        recordOutput("ExitVelocity", currentShot.exitVelocityMps);

        logTrajectoryDebug();
    }

    private void logTrajectoryDebug() {
        TrajectoryResult trajResult = shooterSystem.getLastTrajectoryResult();
        if (trajResult == null) return;

        recordOutput("Trajectory/Status", trajResult.getStatus().name());
        recordOutput("Trajectory/StatusMessage", trajResult.getStatusMessage());
        recordOutput("Trajectory/Confidence", trajResult.getConfidenceScore());

        if (trajResult.isSuccess()) {
            recordOutput("Trajectory/PitchDeg", trajResult.getPitchAngleDegrees());
            recordOutput("Trajectory/YawAdjustDeg", trajResult.getYawAdjustmentDegrees());
            recordOutput("Trajectory/Velocity", trajResult.getRequiredVelocityMps());
            recordOutput("Trajectory/TimeOfFlight", trajResult.getTimeOfFlightSeconds());
            recordOutput("Trajectory/MaxHeight", trajResult.getMaxHeightMeters());
            recordOutput("Trajectory/Margin", trajResult.getMarginOfErrorMeters());
            recordOutput("Trajectory/RPM", trajResult.getRecommendedRpm());

            List<Pose3d> flightPath = trajResult.getFlightPath();
            if (!flightPath.isEmpty()) {
                Logger.recordOutput("ExampleShooter/Trajectory/FlightPath",
                        flightPath.toArray(new Pose3d[0]));

                double[] pathX = new double[flightPath.size()];
                double[] pathY = new double[flightPath.size()];
                double[] pathZ = new double[flightPath.size()];
                for (int i = 0; i < flightPath.size(); i++) {
                    pathX[i] = flightPath.get(i).getX();
                    pathY[i] = flightPath.get(i).getY();
                    pathZ[i] = flightPath.get(i).getZ();
                }
                recordOutput("Trajectory/PathX", pathX);
                recordOutput("Trajectory/PathY", pathY);
                recordOutput("Trajectory/PathZ", pathZ);
                recordOutput("Trajectory/PathLength", flightPath.size());
            }
        }

        SolveDebugInfo debug = trajResult.getDebugInfo();
        if (debug != null) {
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
            recordOutput("Debug/DetailedTable", debug.getDetailedTable());

            List<SolveDebugInfo.CandidateInfo> accepted = debug.getAcceptedCandidates();
            double[] accPitch = new double[accepted.size()];
            double[] accScore = new double[accepted.size()];
            double[] accTOF = new double[accepted.size()];
            double[] accMaxH = new double[accepted.size()];
            for (int i = 0; i < accepted.size(); i++) {
                accPitch[i] = accepted.get(i).getPitchDegrees();
                accScore[i] = accepted.get(i).getScore();
                accTOF[i] = accepted.get(i).getTimeOfFlight();
                accMaxH[i] = accepted.get(i).getMaxHeight();
            }
            recordOutput("Debug/AcceptedPitches", accPitch);
            recordOutput("Debug/AcceptedScores", accScore);
            recordOutput("Debug/AcceptedTOF", accTOF);
            recordOutput("Debug/AcceptedMaxHeight", accMaxH);

            List<SolveDebugInfo.CandidateInfo> all = debug.getCandidates();
            double[] allPitch = new double[all.size()];
            double[] allClosest = new double[all.size()];
            String[] allStatus = new String[all.size()];
            for (int i = 0; i < all.size(); i++) {
                allPitch[i] = all.get(i).getPitchDegrees();
                allClosest[i] = all.get(i).getClosestApproach();
                allStatus[i] = all.get(i).getRejection().name();
            }
            recordOutput("Debug/AllPitches", allPitch);
            recordOutput("Debug/AllClosest", allClosest);
            recordOutput("Debug/AllStatus", allStatus);
        } else {
            recordOutput("Debug/Enabled", false);
        }
    }

    /**
     * Recalculates the shot using the current robot pose and target position.
     * Called automatically when tracking is enabled.
     */
    private void updateShot() {
        Pose2d pose = poseSupplier.get();

        Rotation2d rot = pose.getRotation();
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;

        double dx = targetPosition.getX() - shooterX;
        double dy = targetPosition.getY() - shooterY;
        double yawRad = Math.atan2(dy, dx);
        lastDistanceMeters = Math.hypot(dx, dy);
        targetYawDegrees = Math.toDegrees(yawRad);

        double vx = 0, vy = 0;
        if (chassisSpeedsSupplier != null) {
            ChassisSpeeds speeds = chassisSpeedsSupplier.get();
            vx = speeds.vxMetersPerSecond;
            vy = speeds.vyMetersPerSecond;
        }

        double measuredRpm = currentRpmSupplier != null ? currentRpmSupplier.get() : 0;

        shooterSystem.setSolverInputTemplate(
            ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterHeightMeters)
                .shooterYawRadians(yawRad)
                .targetPositionMeters(targetPosition.getX(), targetPosition.getY(), targetPosition.getZ())
                .targetRadiusMeters(0.53)
                .includeAirResistance(true)
                .robotVelocity(vx, vy)
        );

        currentShot = shooterSystem.calculate(lastDistanceMeters, measuredRpm, vx, vy, yawRad);

        recordOutput("RobotX", pose.getX());
        recordOutput("RobotY", pose.getY());
        recordOutput("ShooterX", shooterX);
        recordOutput("ShooterY", shooterY);
    }

    /**
     * Manual shot calculation (e.g. for autonomous preset positions).
     */
    public void calculateShot(double shooterX, double shooterY, double shooterZ,
                              double targetX, double targetY, double targetZ) {
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
        lastDistanceMeters = Math.hypot(dx, dy);
        targetYawDegrees = Math.toDegrees(Math.atan2(dy, dx));
        currentShot = shooterSystem.calculate(lastDistanceMeters);
    }


    /** Spin up the flywheel to the current target RPM. */
    public Command spinUp() {
        return run(() -> {
            if (currentShot.valid && currentShot.rpm > 0) {
                flywheelLeader.set(Math.min(currentShot.rpm / 6000.0, 1.0));
            }
        });
    }

    /** Stop the flywheel. */
    public Command stopCommand() {
        return runOnce(() -> {
            currentShot = ShotParameters.invalid("Stopped");
            flywheelLeader.stop();
        });
    }

    /** Switch to the next shot mode (cycles through modes). */
    public Command cycleModeCommand() {
        return runOnce(() -> {
            ShotMode[] modes = ShotMode.values();
            int next = (shooterSystem.getMode().ordinal() + 1) % modes.length;
            shooterSystem.setMode(modes[next]);
            System.out.println("Shot mode: " + modes[next]);
        });
    }

    /** Shoot a ball in simulation (FuelSim). */
    public Command shootBallSimCommand() {
        return runOnce(this::shootBallSim);
    }


    private void shootBallSim() {
        if (poseSupplier == null) return;

        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds speeds = chassisSpeedsSupplier != null ? chassisSpeedsSupplier.get() : new ChassisSpeeds();

        double launchSpeed = currentShot.valid ? currentShot.exitVelocityMps : 15.0;
        double pitchRad = currentShot.valid ? currentShot.getPitchRadians() : Math.toRadians(55);
        double yawRad = Math.toRadians(targetYawDegrees);
        if (!currentShot.valid) yawRad = robotPose.getRotation().getRadians();

        Rotation2d rot = robotPose.getRotation();
        double wx = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double wy = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        Translation3d pos = new Translation3d(robotPose.getX() + wx, robotPose.getY() + wy, shooterHeightMeters);

        double hSpeed = launchSpeed * Math.cos(pitchRad);
        Translation3d vel = new Translation3d(
                hSpeed * Math.cos(yawRad) + speeds.vxMetersPerSecond,
                hSpeed * Math.sin(yawRad) + speeds.vyMetersPerSecond,
                launchSpeed * Math.sin(pitchRad));

        FuelSim.getInstance().spawnFuel(pos, vel);
        System.out.printf("Shot! %.1fm/s @ %.1f° [%s]%n", launchSpeed, currentShot.pitchDegrees, currentShot.source);
    }

    public double getTargetRpm()          { return currentShot.rpm; }
    public double getTargetPitchDegrees() { return currentShot.pitchDegrees; }
    public double getTargetYawDegrees()   { return targetYawDegrees; }
    public boolean hasValidShot()         { return currentShot.valid; }
    public ShotParameters getCurrentShot(){ return currentShot; }
    public ShooterSystem getShooterSystem() { return shooterSystem; }

    /** Check if the flywheel is at speed and the shot is safe. */
    public boolean isReadyToFire() {
        double rpm = currentRpmSupplier != null ? currentRpmSupplier.get() : 0;
        return shooterSystem.isReadyToFire(rpm);
    }


    @Override
    public Sendable log() {
        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("ExampleShooter");
                builder.addDoubleProperty("TargetRPM", () -> currentShot.rpm, null);
                builder.addDoubleProperty("TargetPitchDeg", () -> currentShot.pitchDegrees, null);
                builder.addDoubleProperty("TargetYawDeg", () -> targetYawDegrees, null);
                builder.addBooleanProperty("HasValidShot", () -> currentShot.valid, null);
                builder.addStringProperty("ShotSource", () -> currentShot.source.name(), null);
                builder.addStringProperty("Mode", () -> shooterSystem.getMode().name(), null);
                builder.addDoubleProperty("Distance", () -> lastDistanceMeters, null);
            }
        };
    }
}
