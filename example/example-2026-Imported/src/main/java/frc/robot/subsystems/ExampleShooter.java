package frc.robot.subsystems;

import ca.team4308.absolutelib.math.trajectories.shooter.*;
import ca.team4308.absolutelib.math.trajectories.*;
import ca.team4308.absolutelib.math.trajectories.gamepiece.*;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig.WheelArrangement;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelSimulator;
import ca.team4308.absolutelib.math.trajectories.flywheel.WheelMaterial;
import ca.team4308.absolutelib.math.trajectories.motor.FRCMotors;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Util.FuelSim;

import java.util.List;
import java.util.function.Supplier;

public class ExampleShooter extends AbsoluteSubsystem {

    private final TalonFX flywheelLeader;

    private final ShooterSystem shooterSystem;
    private final TrajectorySolver solver;

    private final ca.team4308.absolutelib.network.task.client.CoprocessorClient coprocessorClient;
    private Thread coprocessorThread;
    private ca.team4308.absolutelib.network.task.client.TaskHandle<TrajectoryResponse> currentTaskHandle;
    private TrajectoryResponse lastCoprocessorResponse = null;

    private final CoprocessorClient lossyCoprocessorClient;
    private Thread lossyCoprocessorThread;

    private final edu.wpi.first.networktables.DoubleSubscriber pitchMinSub;
    private final edu.wpi.first.networktables.DoubleSubscriber pitchMaxSub;
    private final edu.wpi.first.networktables.DoubleSubscriber rpmMinSub;
    private final edu.wpi.first.networktables.DoubleSubscriber rpmMaxSub;
    private final edu.wpi.first.networktables.DoubleSubscriber shooterHeightSub;
    private final edu.wpi.first.networktables.DoubleSubscriber rpmVelocityFactorSub;
    private final edu.wpi.first.networktables.DoubleSubscriber distanceMinSub;
    private final edu.wpi.first.networktables.DoubleSubscriber distanceMaxSub;
    private final edu.wpi.first.networktables.IntegerSubscriber configVersionSub;

    private ShotParameters currentShot = ShotParameters.invalid("Not yet calculated");
    private double targetYawDegrees = 0.0;
    private double lastDistanceMeters = 0.0;
    private double lastComputationTimeMs = 0.0;

    private Supplier<Pose2d> poseSupplier = null;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier = null;
    private Supplier<Double> currentRpmSupplier = null;

    private double shooterHeightMeters = 0.5;
    private Translation2d shooterOffset = new Translation2d(0.1, 0.1);
    private Translation3d targetPosition = new Translation3d(0, 0.0, 0);
    private boolean trackingEnabled = true;
    private boolean loggingEnabled = true;

    public ExampleShooter() {
        super();

        flywheelLeader = new TalonFX(40);

        ShooterConfig shooterConfig = ShooterConfig.builder()
                .pitchLimits(47.5, 82.5) // From 90 Degrees (parallel to ground)  so to find the angle if the min and max is 10 and 20 you'd do 90 - 10 , as 0 in this is parrel to the ground and we are shooting upwards so we subtract from 90
                .rpmLimits(0, 6000) // Change Per Motor, So Kraken X60 is roughly 6.1k RPM MAX but we set 6k for some headroom always leave around 
                .rpmToVelocityFactor(0.01532) // This Factor can be modeled or found experimentally, it converts RPM to exit velocity (m/s) this is used for feedforward and safety checks, so its important to be accurate in our case this is a simple guess based on 4 inch wheels and some slip, but ideally you would calculate this based on your wheel diameter, gear ratio, and slip ratio
                .distanceLimits(0.5, 12.0) // Define and Min and Max shot, as if we try to caclulate a shot outside of limits its a waste of resources 
                .rpmFeedbackThreshold(25.0) // the thershold in which RPM is considered close enough to target for feedback compensation to be applied
                .rpmAbortThreshold(500.0) // If we lose to much RPM per SHOT wait for the RPM to recover 
                .pitchCorrectionPerRpmDeficit(0.005) // Per rpmFeedback (25) how much to adjust pitch (deg) to compensate for RPM deficit
                .movingCompensationGain(1) // Total Gain this is a ending multiplier, set to 0 if the movement compensation should be disabled
                .movingIterations(5) // Change to 10 for more aggressive compensation (may cause lag) 
                .safetyMaxExitVelocity(Double.POSITIVE_INFINITY) // No safety limit for testing; set to real max velocity irl 
                .build();

        ShotLookupTable table = new ShotLookupTable()
                .addEntry(1.3, 81.5, 1700.0)
                .addEntry(1.6, 90 - 12.5, 1750)
                .addEntry(1.9, 90 - 13.5, 1780)
                .addEntry(2.3, 90 - 14.5, 1830.0)
                .addEntry(2.6, 90 - 15.5, 1890.0)
                .addEntry(3.3, 90 - 16.5, 1980.0)
                .addEntry(3.9, 90 - 17, 2080.0)
                .addEntry(4.3, 90 - 18, 2160.0)
                .addEntry(4.6, 90 - 19, 2300.0);

        GamePiece gamePiece = GamePieces.REBUILT_2026_BALL;
        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.highAccuracy()
                .toBuilder()
                .minPitchDegrees(47.5)
                .maxPitchDegrees(82.5)
                .build();

        solver = new TrajectorySolver(gamePiece, solverConfig);
        solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);
        solver.addTuningPoint(table);
        
        FlywheelConfig flywheelConfig = FlywheelConfig.builder()
                .name("Example 2026 Shooter")
                .arrangement(WheelArrangement.SINGLE)
                .wheelDiameterInches(4.0)
                .material(WheelMaterial.VERY_HARD)
                .compressionRatio(0.10)
                .motor(FRCMotors.KRAKEN_X60)
                .motorsPerWheel(2)
                .gearRatio(1.0)
                .build();
        solver.setFlywheel(flywheelConfig);
        shooterSystem = new ShooterSystem(shooterConfig, table, solver);
        shooterSystem.setMode(ShotMode.SOLVER_ONLY);

        solver.setDebugEnabled(true); // Disable irl alot of logging is bad for radio 

        String coprocessorIp = RobotBase.isSimulation() ? "127.0.0.1" : "10.43.8.77";
        coprocessorClient = new ca.team4308.absolutelib.network.task.client.CoprocessorClient(coprocessorIp, 5802);
        System.out.println("Starting coprocessor client thread for " + coprocessorIp);
        coprocessorThread = new Thread(coprocessorClient);
        coprocessorThread.setDaemon(true);
        coprocessorThread.start();

    lossyCoprocessorClient = new CoprocessorClient(coprocessorIp, 5801, true);
    System.out.println("Starting lossy coprocessor client thread for " + coprocessorIp);
    lossyCoprocessorThread = new Thread(lossyCoprocessorClient);
    lossyCoprocessorThread.setDaemon(true);
    lossyCoprocessorThread.start();

        edu.wpi.first.networktables.NetworkTableInstance nt = edu.wpi.first.networktables.NetworkTableInstance.getDefault();
        edu.wpi.first.networktables.NetworkTable table2 = nt.getTable("TrajectoryCoprocessor");
        
        pitchMinSub = table2.getDoubleTopic("Config/ShooterPitchMinDeg").subscribe(47.5);
        pitchMaxSub = table2.getDoubleTopic("Config/ShooterPitchMaxDeg").subscribe(82.5);
        rpmMinSub = table2.getDoubleTopic("Config/ShooterRpmMin").subscribe(0.0);
        rpmMaxSub = table2.getDoubleTopic("Config/ShooterRpmMax").subscribe(6000.0);
        shooterHeightSub = table2.getDoubleTopic("Config/ShooterHeightMeters").subscribe(0.5);
        rpmVelocityFactorSub = table2.getDoubleTopic("Config/RpmVelocityFactor").subscribe(0.01532);
        distanceMinSub = table2.getDoubleTopic("Config/DistanceMinMeters").subscribe(0.5);
        distanceMaxSub = table2.getDoubleTopic("Config/DistanceMaxMeters").subscribe(12.0);
        configVersionSub = table2.getIntegerTopic("Status/ActiveConfigVersionId").subscribe(0);
    }

    public void setPoseSupplier(Supplier<Pose2d> supplier) {
        this.poseSupplier = supplier;
    }

    public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> supplier) {
        this.chassisSpeedsSupplier = supplier;
    }

    public void setCurrentRpmSupplier(Supplier<Double> supplier) {
        this.currentRpmSupplier = supplier;
    }

    public void setShooterHeight(double meters) {
        this.shooterHeightMeters = meters;
    }

    public void setShooterOffset(Translation2d offset) {
        this.shooterOffset = offset;
    }

    public void setTarget(double x, double y, double z) {
        this.targetPosition = new Translation3d(x, y, z);
    }

    public void setTrackingEnabled(boolean enabled) {
        this.trackingEnabled = enabled;
    }

    public void setLoggingEnabled(boolean enabled) {
        this.loggingEnabled = enabled;
    }

    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    public void setPitchLimits(double min, double max) {
    }

    /**
     * Change the shot mode at runtime (e.g. from dashboard or button).
     */
    public void setMode(ShotMode mode) {
        shooterSystem.setMode(mode);
    }

    public ShotMode getMode() {
        return shooterSystem.getMode();
    }

    /**
     * Set manual override values for MANUAL mode.
     */
    public void setManualOverride(double pitchDegrees, double rpm) {
        shooterSystem.setManualOverride(pitchDegrees, rpm);
    }

    @Override
    public void periodic() {
        double measuredRpm = RobotBase.isReal() ? flywheelLeader.getVelocity().getValueAsDouble() * 60.0 : currentShot.rpm;

        if (trackingEnabled && poseSupplier != null) {
            updateShot();
        }

        if (currentShot.valid && currentShot.rpm > 0) {
            flywheelLeader.set(Math.min(currentShot.rpm / 6000.0, 1.0));
        }

        if (loggingEnabled) {
            ShooterSystem.ShooterTelemetry telemetry = shooterSystem.getSystemTelemetry(measuredRpm);

            recordOutput("Shooter/Mode", telemetry.mode.name());
            recordOutput("Shooter/Source", telemetry.source.name());
            recordOutput("Shooter/SourceDetail", telemetry.sourceDetail);
            recordOutput("Shooter/Distance", telemetry.distanceMeters);
            recordOutput("Shooter/TargetRPM", telemetry.targetRpm);
            recordOutput("Shooter/TargetPitchDeg", telemetry.targetPitchDegrees);
            recordOutput("Shooter/IsValid", telemetry.isValid);
            recordOutput("Shooter/IsReady", telemetry.isReady);

            if (telemetry.safetyResult != null) {
                recordOutput("Shooter/Safety/Safe", telemetry.safetyResult.safe);
                recordOutput("Shooter/Safety/Reason", telemetry.safetyResult.reason);
            }

            Pose3d goalPose = new Pose3d(targetPosition, new Rotation3d());
            Logger.recordOutput("ExampleShooter/GoalPose3d", goalPose);

            recordOutput("TargetYawDeg", targetYawDegrees);
            recordOutput("MeasuredRPM", measuredRpm);
            recordOutput("RpmDeficit", telemetry.targetRpm - measuredRpm);
        }

        logTrajectoryDebug();
    }

    private void logTrajectoryDebug() {
        if (!loggingEnabled) {
            return;
        }
        TrajectoryResult trajResult = shooterSystem.getLastTrajectoryResult();
        if (trajResult == null) {
            return;
        }

        recordOutput("Trajectory/Trace/SourceMode", trajResult.getSolveModeUsed().name());
        recordOutput("Trajectory/Trace/TimeMs", trajResult.getComputationTimeMs());
        recordOutput("Trajectory/Trace/Iterations", trajResult.getIterations());

        recordOutput("Trajectory/Status", trajResult.getStatus().name());
        recordOutput("Trajectory/StatusMessage", trajResult.getStatusMessage());

        if (trajResult.isSuccess()) {
            recordOutput("Trajectory/PitchDeg", trajResult.getPitchAngleDegrees());
            recordOutput("Trajectory/YawAdjustDeg", trajResult.getYawAdjustmentDegrees());
            recordOutput("Trajectory/Velocity", trajResult.getRequiredVelocityMps());
            recordOutput("Trajectory/TimeOfFlight", trajResult.getTimeOfFlightSeconds());
            recordOutput("Trajectory/MaxHeight", trajResult.getMaxHeightMeters());
            recordOutput("Trajectory/Margin", trajResult.getMarginOfErrorMeters());
            recordOutput("Trajectory/RPM", trajResult.getRecommendedRpm());

            FlywheelSimulator.SimulationResult flywheelSim = trajResult.getFlywheelSimulation();
            if (flywheelSim != null) {
                recordOutput("Flywheel/ExitVelocityMps", flywheelSim.exitVelocityMps);
                recordOutput("Flywheel/MotorPowerPercent", flywheelSim.motorPowerPercent);
                recordOutput("Flywheel/RequiredMotorRpm", flywheelSim.requiredMotorRpm);
                recordOutput("Flywheel/RequiredWheelRpm", flywheelSim.requiredWheelRpm);
                recordOutput("Flywheel/SpinUpTimeSeconds", flywheelSim.spinUpTimeSeconds);
                recordOutput("Flywheel/CurrentDrawAmps", flywheelSim.currentDrawAmps);
                recordOutput("Flywheel/StoredEnergyJoules", flywheelSim.storedEnergyJoules);
                recordOutput("Flywheel/BallSpinRpm", flywheelSim.ballSpinRpm);
                recordOutput("Flywheel/ContactTimeMs", flywheelSim.contactTimeMs);
                recordOutput("Flywheel/EnergyEfficiency", flywheelSim.energyTransferEfficiency);
                recordOutput("Flywheel/SlipRatio", flywheelSim.slipRatio);
                recordOutput("Flywheel/IsAchievable", flywheelSim.isAchievable);
                recordOutput("Flywheel/LimitingFactor", flywheelSim.limitingFactor);
            }

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

        List<Pose3d> lossyPath = lossyCoprocessorClient.getLatestFlightPath();
        if (lossyPath != null && !lossyPath.isEmpty()) {
            Logger.recordOutput("ExampleShooter/Coprocessor/FlightPath",
                    lossyPath.toArray(new Pose3d[0]));

            double[] lossyX = new double[lossyPath.size()];
            double[] lossyY = new double[lossyPath.size()];
            double[] lossyZ = new double[lossyPath.size()];
            for (int i = 0; i < lossyPath.size(); i++) {
                lossyX[i] = lossyPath.get(i).getX();
                lossyY[i] = lossyPath.get(i).getY();
                lossyZ[i] = lossyPath.get(i).getZ();
            }
            recordOutput("Trajectory/LossyPathX", lossyX);
            recordOutput("Trajectory/LossyPathY", lossyY);
            recordOutput("Trajectory/LossyPathZ", lossyZ);
            recordOutput("Trajectory/LossyPathLength", lossyPath.size());
        }

        SolveDebugInfo debug = trajResult.getDebugInfo();
        if (debug != null) {
            recordOutput("Debug/Enabled", false);
            recordOutput("Debug/TotalTested", debug.getTotalTested());
            recordOutput("Debug/Accepted", debug.getAcceptedCount());
            recordOutput("Debug/TotalRejected", debug.getTotalRejected());
            recordOutput("Debug/RejectedCollision", debug.getRejectedCollisionCount());
            recordOutput("Debug/RejectedArcTooLow", debug.getRejectedArcTooLowCount());
            recordOutput("Debug/RejectedClearance", debug.getRejectedClearanceCount());
            recordOutput("Debug/RejectedMiss", debug.getRejectedMissCount());
            recordOutput("Debug/RejectedFlyover", debug.getRejectedFlyoverCount());
            recordOutput("Debug/BestMissDistance", debug.getBestMissDistance());
            recordOutput("Debug/BestPitchDeg", debug.getBestPitchDegrees());
            recordOutput("Debug/Summary", debug.getSummary());
            recordOutput("Debug/DetailedTable", debug.getDetailedTable());

            List<SolveDebugInfo.CandidateInfo> accepted = debug.getAcceptedCandidates();
            double[] accPitch = new double[accepted.size()];
            double[] accMiss = new double[accepted.size()];
            double[] accTOF = new double[accepted.size()];
            double[] accMaxH = new double[accepted.size()];
            for (int i = 0; i < accepted.size(); i++) {
                accPitch[i] = accepted.get(i).getPitchDegrees();
                accMiss[i] = accepted.get(i).getMissDistance();
                accTOF[i] = accepted.get(i).getTimeOfFlight();
                accMaxH[i] = accepted.get(i).getMaxHeight();
            }
            recordOutput("Debug/AcceptedPitches", accPitch);
            recordOutput("Debug/AcceptedMissDistance", accMiss);
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

    private void updateShot() {
        updateConfigFromNT4();
        
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

        double vx = 0, vy = 0, omega = 0;
        if (chassisSpeedsSupplier != null) {
            ChassisSpeeds speeds = chassisSpeedsSupplier.get();
            vx = speeds.vxMetersPerSecond;
            vy = speeds.vyMetersPerSecond;
            omega = speeds.omegaRadiansPerSecond;
        }

        double measuredRpm = currentRpmSupplier != null ? currentRpmSupplier.get() : 0;

        TrajectoryRequest req = new TrajectoryRequest();
        req.timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        req.robotX = shooterX;
        req.robotY = shooterY;
        req.robotHeadingRad = yawRad;
        req.vxMps = vx;
        req.vyMps = vy;
        req.omegaRadPerSecond = omega;
        req.targetX = targetPosition.getX();
        req.targetY = targetPosition.getY();
        req.targetZ = targetPosition.getZ();
        req.currentRpm = measuredRpm;

    lossyCoprocessorClient.setRequest(req);

        long startTime = System.nanoTime();
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        coprocessorClient.pruneStaleTasks(now, 1.0);

        TrajectoryResponse res = null;
        boolean isFreshAndValid = false;

        if (currentTaskHandle != null && currentTaskHandle.isDone() && currentTaskHandle.isSuccess()) {
            res = currentTaskHandle.get();
            if (res != null) {
                lastCoprocessorResponse = res;
            }
        }

        if (lastCoprocessorResponse != null) {
            isFreshAndValid = (now - lastCoprocessorResponse.timestamp) < 0.5;
            if (isFreshAndValid) {
                res = lastCoprocessorResponse;
            }
        }

        if (currentTaskHandle == null || currentTaskHandle.isDone() || currentTaskHandle.isStale(now, 0.5)) {
            currentTaskHandle = coprocessorClient.submitTask("TRAJECTORY_SOLVE", req, TrajectoryResponse.class, now);
        }

        // Check if coprocessor has a fresh valid response
        if (isFreshAndValid) {
            recordOutput("Shooter/CalculatedOnCoProcessor", true);
            currentShot = res.valid
                    ? new ShotParameters(res.pitchDegrees, res.rpm, 0.0, lastDistanceMeters, Math.toRadians(res.yawDegrees), ShotParameters.Source.SOLVER)
                    : ShotParameters.invalid(res.status);

            // Sync fallback system 
            shooterSystem.setManualOverride(res.pitchDegrees, res.rpm);
            if (loggingEnabled) {
                recordOutput("Shooter/FallbackActive", false);
            }
        } else {
            if (loggingEnabled) {
                recordOutput("Shooter/FallbackActive", true);
                if (res != null) {
                    recordOutput("Shooter/FallbackReason_StaleTime", edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - res.timestamp);
                }
            }
            // Fallback to RoboRIO solving
            shooterSystem.setSolverInput(
                    ShotInput.builder()
                            .shooterPositionMeters(shooterX, shooterY, shooterHeightMeters)
                            .shooterYawRadians(yawRad)
                            .targetPositionMeters(targetPosition.getX(), targetPosition.getY(), targetPosition.getZ())
                            .targetRadiusMeters(0.45)
                            .includeAirResistance(true)
                            .robotVelocity(vx, vy, omega, shooterSystem.getConfig().getShooterRadiusMeters())
                            .build()
            );
            recordOutput("Shooter/CalculatedOnCoProcessor", false);
            currentShot = shooterSystem.calculate(lastDistanceMeters, measuredRpm, vx, vy, omega, yawRad);
        }
        long endTime = System.nanoTime();
        lastComputationTimeMs = (endTime - startTime) / 1_000_000.0;

        if (loggingEnabled && res != null) {
            recordOutput("Shooter/CoprocessorRTT_ms", (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - res.timestamp) * 1000.0);
        }

        if (loggingEnabled) {
            recordOutput("RobotX", pose.getX());
            recordOutput("RobotY", pose.getY());
            recordOutput("ShooterX", shooterX);
            recordOutput("ShooterY", shooterY);
        }
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

    /**
     * Spin up the flywheel to the current target RPM.
     */
    public Command spinUp() {
        return run(() -> {
            if (currentShot.valid && currentShot.rpm > 0) {
                flywheelLeader.set(Math.min(currentShot.rpm / 6000.0, 1.0));
            }
        });
    }

    /**
     * Stop the flywheel.
     */
    public Command stopCommand() {
        return runOnce(() -> {
            currentShot = ShotParameters.invalid("Stopped");
            flywheelLeader.set(0);
        });
    }

    /**
     * Switch to the next shot mode (cycles through modes).
     */
    public Command cycleModeCommand() {
        return runOnce(() -> {
            ShotMode[] modes = ShotMode.values();
            int next = (shooterSystem.getMode().ordinal() + 1) % modes.length;
            shooterSystem.setMode(modes[next]);
            System.out.println("Shot mode: " + modes[next]);
        });
    }

    /**
     * Shoot a ball in simulation (FuelSim).
     */
    public Command shootBallSimCommand() {
        return runOnce(this::shootBallSim);
    }

    private void shootBallSim() {
        if (poseSupplier == null) {
            return;
        }

        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds speeds = chassisSpeedsSupplier != null ? chassisSpeedsSupplier.get() : new ChassisSpeeds();
        TrajectoryResult trajResult = shooterSystem.getLastTrajectoryResult();
        if (trajResult == null || !trajResult.isSuccess()) {
            System.out.println("Cannot shoot: no valid trajectory");
            return;
        }

        double launchSpeed = trajResult.getRequiredVelocityMps();
        double pitchRad = Math.toRadians(trajResult.getPitchAngleDegrees());
        double yawRad = Math.toRadians(targetYawDegrees) + trajResult.getYawAdjustmentRadians();

        Rotation2d rot = robotPose.getRotation();
        double wx = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double wy = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        Translation3d pos = new Translation3d(robotPose.getX() + wx, robotPose.getY() + wy, shooterHeightMeters);

        double hSpeed = launchSpeed * Math.cos(pitchRad);
        Translation3d vel = new Translation3d(
                hSpeed * Math.cos(yawRad) + speeds.vxMetersPerSecond,
                hSpeed * Math.sin(yawRad) + speeds.vyMetersPerSecond,
                launchSpeed * Math.sin(pitchRad));

        List<Pose3d> predictedPath = trajResult.getFlightPath();
        FuelSim.getInstance().spawnFuelTracked(pos, vel, predictedPath);
    }

    public double getTargetRpm() {
        return currentShot.rpm;
    }

    public double getTargetPitchDegrees() {
        return currentShot.pitchDegrees;
    }

    public double getTargetYawDegrees() {
        return targetYawDegrees;
    }

    public boolean hasValidShot() {
        return currentShot.valid;
    }

    public ShotParameters getCurrentShot() {
        return currentShot;
    }

    public ShooterSystem getShooterSystem() {
        return shooterSystem;
    }

    private void updateConfigFromNT4() {
        shooterHeightMeters = shooterHeightSub.get();
        
        double minPitch = pitchMinSub.get();
        double maxPitch = pitchMaxSub.get();
        double minRpm = rpmMinSub.get();
        double maxRpm = rpmMaxSub.get();
        double rpmFactor = rpmVelocityFactorSub.get();
        double distMin = distanceMinSub.get();
        double distMax = distanceMaxSub.get();
        long configVersion = configVersionSub.get();
        
        recordOutput("Config/ActiveVersion", configVersion);
        recordOutput("Config/ShooterHeight", shooterHeightMeters);
        recordOutput("Config/PitchRange", new double[]{minPitch, maxPitch});
        recordOutput("Config/RPMRange", new double[]{minRpm, maxRpm});
    }

    /**
     * Check if the flywheel is at speed and the shot is safe.
     */
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
