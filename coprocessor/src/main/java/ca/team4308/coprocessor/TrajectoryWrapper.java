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
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;

public class TrajectoryWrapper {

    private final TrajectorySolver solver;
    private final ShooterSystem shooterSystem;
    private final OutputSmoother smoother;

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
                        .shooterPositionMeters(req.robotX, req.robotY, 0.5) // Example height
                        .shooterYawRadians(yawRadians)
                        .targetPositionMeters(req.targetX, req.targetY, req.targetZ)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .robotVelocity(req.vxMps, req.vyMps)
                        .build()
        );

        ShotParameters shot = shooterSystem.calculate(distanceMeters, req.currentRpm, req.vxMps, req.vyMps, yawRadians);

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
                                .robotVelocity(req.vxMps, req.vyMps)
                                .build()
                    );
                    shot = shooterSystem.calculate(pDistance, req.currentRpm, req.vxMps, req.vyMps, pYaw);
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
}
