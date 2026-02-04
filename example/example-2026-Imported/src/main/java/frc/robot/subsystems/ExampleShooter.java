package frc.robot.subsystems;

import ca.team4308.absolutelib.math.trajectories.*;
import ca.team4308.absolutelib.math.trajectories.flywheel.*;
import ca.team4308.absolutelib.math.trajectories.gamepiece.*;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

/**
 * Example shooter subsystem demonstrating the 2026 trajectory calculation
 * system. Uses FlywheelGenerator to find optimal shot configurations and
 * TrajectorySolver for real-time shot calculations.
 */
public class ExampleShooter extends AbsoluteSubsystem {

    // Hardware
    private final MotorWrapper flywheelLeader;
    private final MotorWrapper flywheelFollower;

    // Trajectory System
    private final TrajectorySolver solver;
    private final FlywheelConfig flywheelConfig;
    private final GamePiece gamePiece;

    // State
    private double targetRpm = 0.0;
    private double targetPitchDegrees = 0.0;
    private boolean hasValidShot = false;
    private ShotCandidate lastShot = null;

    public ExampleShooter() {
        super();

        // Configure flywheel motors (dual over-under configuration)
        flywheelLeader = new MotorWrapper(MotorType.TALONFX, 40);
        flywheelFollower = new MotorWrapper(MotorType.TALONFX, 41);
        flywheelFollower.follow(flywheelLeader);

        // Set up game piece for 2026 REBUILT
        gamePiece = GamePieces.REBUILT_2026_BALL;

        // Create trajectory solver for 2026 game
        solver = TrajectorySolver.forGame2026();

        // Generate optimal flywheel configuration for target velocity range
        FlywheelGenerator generator = new FlywheelGenerator(gamePiece);
        FlywheelGenerator.GenerationResult result = generator.generateAndEvaluate(15.0); // 15 m/s target

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
                    .material(WheelMaterial.GREEN_COMPLIANT)
                    .compressionRatio(0.10)
                    .wheelCount(2)
                    .motor(ca.team4308.absolutelib.math.trajectories.motor.FRCMotors.FALCON_500)
                    .motorsPerWheel(1)
                    .gearRatio(1.0)
                    .build();
        }
    }

    @Override
    public void periodic() {
        // Update motor control based on target RPM
        if (targetRpm > 0) {
            // Convert RPM to percent output (simplified - real code would use velocity PID)
            double percentOutput = Math.min(targetRpm / 6000.0, 1.0);
            flywheelLeader.set(percentOutput);
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
        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterZ)
                .targetPositionMeters(targetX, targetY, targetZ)
                .shotPreference(ShotInput.ShotPreference.FASTEST)
                .build();

        // Find all possible shots
        ShotCandidateList candidates = solver.findAllCandidates(input);

        // Get the fastest achievable shot
        Optional<ShotCandidate> fastest = candidates.getFastest();

        if (fastest.isPresent()) {
            lastShot = fastest.get();
            targetPitchDegrees = lastShot.getPitchAngleDegrees();
            targetRpm = lastShot.getSpeedScore();
            hasValidShot = true;
        } else {
            hasValidShot = false;
            targetRpm = 0;
            targetPitchDegrees = 0;
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

    public boolean hasValidShot() {
        return hasValidShot;
    }

    public ShotCandidate getLastShot() {
        return lastShot;
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
                builder.addBooleanProperty("HasValidShot", () -> hasValidShot, null);
                if (lastShot != null) {
                    builder.addDoubleProperty("TimeOfFlight", () -> lastShot.getTimeOfFlightSeconds(), null);
                    builder.addDoubleProperty("Confidence", () -> lastShot.getAccuracyScore(), null);
                }
            }
        };
    }
}
