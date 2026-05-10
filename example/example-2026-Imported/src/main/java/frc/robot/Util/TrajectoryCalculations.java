package frc.robot.Util;

import java.util.List;
import java.util.function.Supplier;
import java.net.InetAddress;
import com.ctre.phoenix6.hardware.TalonFX;
import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.SolveDebugInfo;
import ca.team4308.absolutelib.math.trajectories.TrajectoryResult;
import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig.WheelArrangement;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelSimulator;
import ca.team4308.absolutelib.math.trajectories.flywheel.WheelMaterial;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;
import ca.team4308.absolutelib.math.trajectories.motor.FRCMotors;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;
import ca.team4308.absolutelib.math.trajectories.shooter.ShooterConfig;
import ca.team4308.absolutelib.math.trajectories.shooter.ShooterSystem;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotLookupTable;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotMode;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotParameters;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public class TrajectoryCalculations extends AbsoluteSubsystem {


    private final ShooterSystem shooterSystem;
    private final TrajectorySolver solver;
    private TalonFX flywheelLeader;
    // Coprocessor connection
    private final ca.team4308.absolutelib.network.task.client.CoprocessorClient coprocessorClient;
    private final Thread coprocessorThread;
    private ca.team4308.absolutelib.network.task.client.TaskHandle<TrajectoryResponse> currentTaskHandle;
    private final String coprocessorIp;
    private double lastPingMs = 0;
    private Thread pingThread;

    private ShotParameters currentShot = ShotParameters.invalid("Not yet calculated");
    private double targetYawDegrees = 0.0;
    private double lastDistanceMeters = 0.0;

    private Supplier<Pose2d> poseSupplier = null;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier = null;
    private Supplier<Double> currentRpmSupplier = null;

    private double shooterHeightMeters = 0.5;
    private Translation2d shooterOffset = new Translation2d(0.1, 0.1);
    private Translation3d targetPosition = FieldConstants.Hub.topCenterPoint;
    private boolean trackingEnabled = true;
    private boolean loggingEnabled = true;

    public TrajectoryCalculations(TalonFX flywheelLeader) {
        super();
        this.flywheelLeader = flywheelLeader;
        
        // Set coprocessor IP
        this.coprocessorIp = "10.43.8.11";
        
        ShooterConfig shooterConfig = ShooterConfig.builder()
                .pitchLimits(47.5, 82.5)
                .rpmLimits(0, 6000) // Kraken xt60 max rpm
                .rpmToVelocityFactor(0.01532)
                .distanceLimits(0.5, 12.0)
                .rpmFeedbackThreshold(25.0)
                .rpmAbortThreshold(500.0)
                .pitchCorrectionPerRpmDeficit(0.005)
                .movingCompensationGain(1)
                .movingIterations(3)
                .safetyMaxExitVelocity(99)
                .build();

        ShotLookupTable LookupTable = new ShotLookupTable();
        LookupTable.addEntry(1.3, 90 - 8.5, 1700.0);
        LookupTable.addEntry(1.6, 90 - 12.5, 1750.0);
        LookupTable.addEntry(1.9, 90 - 13.5, 1780.0);
        LookupTable.addEntry(2.3, 90 - 14.5, 1830.0);
        LookupTable.addEntry(2.6, 90 - 15.5, 1890.0);
        LookupTable.addEntry(2.9, 90 - 16.5, 1980.0);
        LookupTable.addEntry(3.3, 90 - 17.0, 2080.0);
        LookupTable.addEntry(3.6, 90 - 17.5, 2160.0);
        LookupTable.addEntry(3.9, 90 - 18.0, 2180.0);
        LookupTable.addEntry(4.3, 90 - 18.5, 2240.0);
        LookupTable.addEntry(4.6, 90 - 19.0, 2305.0);
        LookupTable.addEntry(4.9, 90 - 19.0, 2380.0);
        LookupTable.addEntry(5.2, 90 - 19.5, 24200.);
        LookupTable.addEntry(5.5, 90 - 20.0, 2480.0);
        LookupTable.addEntry(6.0, 90 - 20.0, 2550.0);
        LookupTable.addEntry(7.0, 90 - 21.0, 2650.0);
        LookupTable.addEntry(8.0, 90 - 22.0, 2750.0);
        LookupTable.addEntry(9.0, 90 - 22.5, 2850.0);
        LookupTable.addEntry(10.0, 90 - 23.0, 2950.0);
        LookupTable.addEntry(11.0, 90 - 23.5, 3050.0);
        LookupTable.addEntry(12.0, 90 - 24.0, 3150.0);
        LookupTable.addEntry(13.0, 90 - 24.5, 3250.0);
        LookupTable.addEntry(14.0, 90 - 25.0, 3350.0);
        LookupTable.addEntry(15.0, 90 - 25.5, 3450.0);
        LookupTable.addEntry(16.0, 90 - 26.0, 3550.0);
        GamePiece gamePiece = GamePieces.REBUILT_2026_BALL;
        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.roboRIOPerformance()
                .toBuilder()
                .minPitchDegrees(47.5)
                .maxPitchDegrees(82.5)
                .build();

        solver = new TrajectorySolver(gamePiece, solverConfig);
        solver.setSolveMode(TrajectorySolver.SolveMode.BISECTION);
        solver.addTuningPoint(LookupTable);
        FlywheelConfig flywheelConfig = FlywheelConfig.builder()
                .name("2026 Shooter")
                .arrangement(WheelArrangement.SINGLE)
                .wheelDiameterInches(4.0)
                .material(WheelMaterial.VERY_HARD)
                .compressionRatio(0.10)
                .motor(FRCMotors.KRAKEN_X60)
                .motorsPerWheel(2)
                .gearRatio(1.0)
                .build();
        solver.setFlywheel(flywheelConfig);
        shooterSystem = new ShooterSystem(shooterConfig, LookupTable, solver);
        shooterSystem.setMode(ShotMode.SOLVER_ONLY);

        solver.setDebugEnabled(true);

        // Connect to coprocessor (always use coprocessor, never local solver)
        // IMPORTANT: Use port 5802 (TaskServer), NOT 5805 (which is HTTP web dashboard)
        coprocessorClient = new ca.team4308.absolutelib.network.task.client.CoprocessorClient(coprocessorIp, 5802);
        System.out.println("Starting coprocessor client thread for " + coprocessorIp + ":5802 (TaskServer)");
        coprocessorThread = new Thread(coprocessorClient);
        coprocessorThread.setDaemon(true);
        coprocessorThread.start();

        // Start ping thread to monitor connection
        startPingThread();
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


    public void setLoggingEnabled(boolean enabled) {
        this.loggingEnabled = enabled;
    }

    public void setUseCoprocessor(boolean enabled) {
        // Always use coprocessor - this method is a no-op
    }

    public boolean isUseCoprocessor() {
        return true;
    }

    private void startPingThread() {
        pingThread = new Thread(() -> {
            while (true) {
                try {
                    long startTime = System.currentTimeMillis();
                    InetAddress address = InetAddress.getByName(coprocessorIp);
                    if (address.isReachable(1000)) {
                        lastPingMs = System.currentTimeMillis() - startTime;
                        Logger.recordOutput("Coprocessor/PingMs", lastPingMs);
                    } else {
                        Logger.recordOutput("Coprocessor/PingMs", -1.0);
                    }
                    Thread.sleep(500); // Ping every 500ms
                } catch (Exception e) {
                    Logger.recordOutput("Coprocessor/PingMs", -1.0);
                }
            }
        });
        pingThread.setDaemon(true);
        pingThread.start();
    }

    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    @Override
    public void periodic() {
        double measuredRpm = RobotBase.isReal() ? flywheelLeader.getVelocity().getValueAsDouble() * 60.0
                : currentShot.rpm;

        if (loggingEnabled) {
            recordOutput("Trajectory/TrackingEnabled", trackingEnabled);
            recordOutput("Trajectory/PoseSupplierNull", poseSupplier == null);
            recordOutput("Trajectory/CurrentShotValid", currentShot.valid);
        }

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

            List<edu.wpi.first.math.geometry.Pose3d> flightPath = trajResult.getFlightPath();
            if (!flightPath.isEmpty()) {
                int sampleRate = Math.max(1, flightPath.size() / 30); // Keep max 30 points
                java.util.ArrayList<edu.wpi.first.math.geometry.Pose3d> sampledPath = new java.util.ArrayList<>();
                for (int i = 0; i < flightPath.size(); i += sampleRate) {
                    sampledPath.add(flightPath.get(i));
                }
                if (flightPath.size() % sampleRate != 0) {
                    sampledPath.add(flightPath.get(flightPath.size() - 1));
                }
                
                recordOutput("Shooter/Trajectory/FlightPath",
                        sampledPath.toArray(new edu.wpi.first.math.geometry.Pose3d[0]));
                
                // Also record all coordinates for detailed analysis
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

        TrajectoryRequest req = new TrajectoryRequest();
        req.timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        req.robotX = shooterX;
        req.robotY = shooterY;
        req.robotHeadingRad = yawRad;
        req.vxMps = vx;
        req.vyMps = vy;
        req.omegaRadPerSecond = chassisSpeedsSupplier != null ? chassisSpeedsSupplier.get().omegaRadiansPerSecond : 0.0;
        req.targetX = targetPosition.getX();
        req.targetY = targetPosition.getY();
        req.targetZ = targetPosition.getZ();
        req.currentRpm = measuredRpm;

        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        coprocessorClient.pruneStaleTasks(now, 1.0);

        // Always submit task to coprocessor (never local solver)
        if (currentTaskHandle == null || currentTaskHandle.isDone() || currentTaskHandle.isStale(now, 0.5)) {
            currentTaskHandle = coprocessorClient.submitTask("TRAJECTORY_SOLVE", req, TrajectoryResponse.class, now);
        }

        if (loggingEnabled) {
          //  System.out.println("Trajectory: coprocessor connected=" + ", request=" + req.robotX + "," + req.robotY);
            if (currentTaskHandle != null) {
                //ystem.out.println("Trajectory: currentTaskHandle done=" + currentTaskHandle.isDone() + " stale="
                       // + currentTaskHandle.isStale(now, 0.5));
            }
        }

        TrajectoryResponse res = null;
        boolean isFreshAndValid = false;

        if (currentTaskHandle != null && currentTaskHandle.isDone() && currentTaskHandle.isSuccess()) {
            res = currentTaskHandle.get();
            if (res != null) {
                isFreshAndValid = (now - res.timestamp) < 0.5;
            }
        }

        // Check if coprocessor has a fresh valid response
        if (isFreshAndValid) {
            if (loggingEnabled) {
                System.out.println(
                        "Trajectory: using coprocessor response, pitch=" + res.pitchDegrees + " rpm=" + res.rpm);
            }
            currentShot = res.valid
                    ? new ShotParameters(res.pitchDegrees, res.rpm, 0.0, lastDistanceMeters,
                            Math.toRadians(res.yawDegrees), ShotParameters.Source.SOLVER)
                    : ShotParameters.invalid(res.status);

            // Sync fallback system
            shooterSystem.setManualOverride(res.pitchDegrees, res.rpm);
            if (loggingEnabled) {
                recordOutput("Shooter/FallbackActive", false);
            }
        } else {
            if (loggingEnabled) {
                //System.out.println("Trajectory: fallback to local solver; coprocessor connected=");
            }
            if (loggingEnabled) {
                recordOutput("Shooter/FallbackActive", true);
                if (res != null) {
                    recordOutput("Shooter/FallbackReason_StaleTime",
                            edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - res.timestamp);
                }
            }
            shooterSystem.setSolverInput(
                    ShotInput.builder()
                            .shooterPositionMeters(shooterX, shooterY, shooterHeightMeters)
                            .shooterYawRadians(yawRad)
                            .targetPositionMeters(targetPosition.getX(), targetPosition.getY(), targetPosition.getZ())
                            .targetRadiusMeters(0.45)
                            .includeAirResistance(true)
                            .robotVelocity(vx, vy)
                            .build());
            currentShot = shooterSystem.calculate(lastDistanceMeters, measuredRpm, vx, vy, yawRad);
        }

        if (loggingEnabled && res != null) {
            recordOutput("Shooter/CoprocessorRTT_ms",
                    (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - res.timestamp) * 1000.0);
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
                builder.setSmartDashboardType("TrajectoryCalculations");
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
