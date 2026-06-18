package ca.team4308.coprocessor;

import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;
import ca.team4308.absolutelib.math.trajectories.shooter.ShooterConfig;
import ca.team4308.absolutelib.math.trajectories.shooter.ShooterSystem;
import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotLookupTable;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotMode;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotParameters;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.WheelMaterial;
import ca.team4308.absolutelib.math.trajectories.motor.FRCMotors;
import ca.team4308.absolutelib.math.trajectories.network.ConfigurationPacket;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;

public class TrajectoryWrapper {

    private final TrajectorySolver solver;
    private final ShooterSystem shooterSystem;
    private final OutputSmoother smoother;
    
    // Active configuration tracking
    private volatile int currentConfigVersionId = 0;
    private volatile double shooterHeightMeters = 0.5;

    public TrajectoryWrapper() {
        smoother = new OutputSmoother(Config.SMOOTHING_EMA_ALPHA, Config.SMOOTHING_RESET_THRESHOLD_DEG);

        // High fidelity configuration for the coprocessor
        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.coProcessor()
                .toBuilder()
                .minPitchDegrees(47.5)
                .maxPitchDegrees(82.5)
                .build();

        GamePiece gamePiece = GamePieces.REBUILT_2026_BALL;
        solver = new TrajectorySolver(gamePiece, solverConfig);
        solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);
        
        FlywheelConfig flywheelConfig = FlywheelConfig.builder()
                .name("Coprocessor 2026 Shooter")
                .arrangement(FlywheelConfig.WheelArrangement.SINGLE)
                .wheelDiameterInches(4.0)
                .material(WheelMaterial.VERY_HARD)
                .compressionRatio(0.10)
                .motor(FRCMotors.KRAKEN_X60)
                .motorsPerWheel(2)
                .gearRatio(1.0)
                .build();
        solver.setFlywheel(flywheelConfig);

        ShooterConfig shooterConfig = ShooterConfig.builder()
                .pitchLimits(47.5, 82.5)
                .rpmLimits(0, 6000)
                .rpmToVelocityFactor(0.00532)
                .distanceLimits(0.5, 12.0)
                .rpmFeedbackThreshold(50.0)
                .rpmAbortThreshold(500.0)
                .pitchCorrectionPerRpmDeficit(0.005)
                .movingCompensationGain(1.0)
                .movingIterations(999)
                .safetyMaxExitVelocity(30.0)
                .build();

        ShotLookupTable table = new ShotLookupTable()
                .addEntry(1.3, 81.5, 1700.0)
                .addEntry(1.6, 90-12.5, 1750)
                .addEntry(1.9, 90-13.5, 1780)
                .addEntry(2.3, 90-14.5, 1830.0)
                .addEntry(2.6, 90-15.5, 1890.0)
                .addEntry(3.3, 90-16.5, 1980.0)
                .addEntry(3.9, 90-17, 2080.0)
                .addEntry(4.3, 90-18, 2160.0)
                .addEntry(4.6, 90-19, 2300.0);

        shooterSystem = new ShooterSystem(shooterConfig, table, solver);
        shooterSystem.setMode(ShotMode.SOLVER_ONLY);
        solver.setDebugEnabled(true);
    }

    public TrajectoryResponse solve(TrajectoryRequest req) {
        double dx = req.targetX - req.robotX;
        double dy = req.targetY - req.robotY;
        double distanceMeters = Math.hypot(dx, dy);
        double yawRadians = Math.atan2(dy, dx);
        
        shooterSystem.setSolverInput(
                ShotInput.builder()
                        .shooterPositionMeters(req.robotX, req.robotY, shooterHeightMeters)
                        .shooterYawRadians(yawRadians)
                        .targetPositionMeters(req.targetX, req.targetY, req.targetZ)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .robotVelocity(req.vxMps, req.vyMps, req.omegaRadPerSecond, shooterSystem.getConfig().getShooterRadiusMeters())
                        .build()
        );

        ShotParameters shot = shooterSystem.calculate(distanceMeters, req.currentRpm, req.vxMps, req.vyMps, req.omegaRadPerSecond, yawRadians);

        // Optional Prediction Loop
        if (Config.PREDICTION_ENABLED && shot.valid) {
            for (int i = 0; i < Config.PREDICTION_ITERATIONS; i++) {
                ca.team4308.absolutelib.math.trajectories.TrajectoryResult trajResult = shooterSystem.getLastTrajectoryResult();
                if (trajResult != null && trajResult.isSuccess()) {
                    double tof = trajResult.getTimeOfFlightSeconds();
                    double predX = req.robotX + req.vxMps * tof;
                    double predY = req.robotY + req.vyMps * tof;
                    
                    double pdx = req.targetX - predX;
                    double pdy = req.targetY - predY;
                    double pDistance = Math.hypot(pdx, pdy);
                    double pYaw = Math.atan2(pdy, pdx);

                    shooterSystem.setSolverInput(
                        ShotInput.builder()
                                .shooterPositionMeters(predX, predY, 0.5) 
                                .shooterYawRadians(pYaw)
                                .targetPositionMeters(req.targetX, req.targetY, req.targetZ)
                                .targetRadiusMeters(0.45)
                                .includeAirResistance(true)
                                .robotVelocity(req.vxMps, req.vyMps, req.omegaRadPerSecond, shooterSystem.getConfig().getShooterRadiusMeters())
                                .build()
                    );
                    shot = shooterSystem.calculate(pDistance, req.currentRpm, req.vxMps, req.vyMps, req.omegaRadPerSecond, pYaw);
                    yawRadians = pYaw;
                }
            }
        }

        TrajectoryResponse res = new TrajectoryResponse();
        res.timestamp = req.timestamp;
        res.valid = shot.valid;
        res.pitchDegrees = shot.pitchDegrees;
        
        ca.team4308.absolutelib.math.trajectories.TrajectoryResult trajResult = shooterSystem.getLastTrajectoryResult();
        if (trajResult != null && trajResult.isSuccess()) {
            res.yawDegrees = Math.toDegrees(yawRadians + trajResult.getYawAdjustmentRadians());
            res.timeOfFlightSec = trajResult.getTimeOfFlightSeconds();
        } else {
            res.yawDegrees = Math.toDegrees(yawRadians);
            res.timeOfFlightSec = 0;
        }

        res.rpm = shot.rpm;
        res.confidence = shot.valid ? 1.0 : 0.0;
        res.status = trajResult != null ? trajResult.getStatusMessage() : (shot.valid ? "OK" : "INVALID");
        res.readyToFire = shooterSystem.isReadyToFire(req.currentRpm);

        // Smooth output if valid
        smoother.process(res);

        return res;
    }
    
    public ShooterSystem getShooterSystem() {
        return shooterSystem;
    }

    public java.util.List<edu.wpi.first.math.geometry.Pose3d> projectActiveFlightPath(TrajectoryRequest req) {
        if (req == null || req.activeRpm <= 0 || !Double.isFinite(req.activePitchDegrees)) {
            return java.util.List.of();
        }

        double exitVelocity = shooterSystem.getConfig().rpmToVelocity(req.activeRpm);
        if (exitVelocity <= 0 || !Double.isFinite(exitVelocity)) {
            return java.util.List.of();
        }

        try {
            double yawRadians = Math.atan2(req.targetY - req.robotY, req.targetX - req.robotX);
            ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion projectileMotion =
                    new ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion(
                            ca.team4308.absolutelib.math.trajectories.physics.AirResistance.withMagnus());
            ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion.TrajectoryResult sim =
                    projectileMotion.simulate(
                            ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces.REBUILT_2026_BALL,
                            req.robotX, req.robotY, shooterHeightMeters,
                            exitVelocity,
                            Math.toRadians(req.activePitchDegrees),
                            yawRadians,
                            0,
                            req.vxMps, req.vyMps,
                            req.targetX, req.targetY, req.targetZ,
                            0.45);

            java.util.List<edu.wpi.first.math.geometry.Pose3d> path = new java.util.ArrayList<>();
            if (sim.trajectory == null) {
                return path;
            }

            for (ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion.TrajectoryState state : sim.trajectory) {
                if (state == null) {
                    break;
                }
                path.add(new edu.wpi.first.math.geometry.Pose3d(
                        new edu.wpi.first.math.geometry.Translation3d(state.x, state.y, state.z),
                        new edu.wpi.first.math.geometry.Rotation3d(
                                0.0,
                                Math.atan2(state.vz, Math.sqrt(state.vx * state.vx + state.vy * state.vy)),
                                Math.atan2(state.vy, state.vx))));
            }

            return path;
        } catch (Exception e) {
            return java.util.List.of();
        }
    }

    public void updateConfiguration(ca.team4308.absolutelib.math.trajectories.network.ConfigurationPacket config) {
        if (config == null) {
            return;
        }

        currentConfigVersionId = config.configVersionId;
        shooterHeightMeters = config.shooterHeightMeters;

        // Update Solver Config
        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.fromDTO(config).build();
        solver.updateConfig(solverConfig);
        
        if (config.solverSolveMode != null) {
            try {
                solver.setSolveMode(TrajectorySolver.SolveMode.valueOf(config.solverSolveMode));
            } catch (Exception e) {}
        }

        // Update Shooter Config
        ShooterConfig shooterConfig = ShooterConfig.Builder.fromDTO(config).build();
        shooterSystem.setConfig(shooterConfig);
        
        if (config.shooterMode != null) {
            try {
                shooterSystem.setMode(ShotMode.valueOf(config.shooterMode));
            } catch (Exception e) {}
        }
        
        shooterSystem.setBlendFactor(config.shooterBlendFactor);

        // Update Lookup Table
        if (config.lookupDistances != null && config.lookupDistances.length > 0) {
            ShotLookupTable newTable = new ShotLookupTable(config.shooterRpmToVelocityFactor);
            for (int i = 0; i < config.lookupDistances.length; i++) {
                if (i < config.lookupPitches.length && i < config.lookupRpms.length) {
                    if (i < config.lookupTofs.length) {
                        newTable.addEntry(config.lookupDistances[i], config.lookupPitches[i], config.lookupRpms[i], config.lookupTofs[i]);
                    } else {
                        newTable.addEntry(config.lookupDistances[i], config.lookupPitches[i], config.lookupRpms[i]);
                    }
                }
            }
            shooterSystem.setLookupTable(newTable);
            solver.addTuningPoint(newTable);
        }

        // Log the configuration update for dashboard sync verification
        System.out.println("=== Configuration Updated (Version " + config.configVersionId + ") ===");
        System.out.println("Shooter height: " + shooterHeightMeters + "m");
    }

    public int getCurrentConfigVersionId() {
        return currentConfigVersionId;
    }
}
