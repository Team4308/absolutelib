package ca.team4308.absolutelib.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelSimulator;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Result of a trajectory solution. Contains all information needed to execute
 * the shot.
 */
public class TrajectoryResult {

    /**
     * Returns a list of Pose3d objects representing the flight path for
     * visualization. This uses the simulated trajectory from the solver. The
     * trajectory naturally terminates when the ball lands at the target (on
     * descent). Returns an empty list if no valid solution exists.
     */
    public List<Pose3d> getFlightPath() {
        // Only provide a path if the solution is successful and input is available
        if (!isSuccess() || input == null) {
            return List.of();
        }

        try {
            ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion projectileMotion
                    = new ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion();
            // Pass targetRadius=0 so the simulation runs the full arc to the ground
            // instead of terminating early at the basket-descent condition.
            ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion.TrajectoryResult simResult = projectileMotion.simulate(
                    gamePiece != null ? gamePiece : ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces.getCurrent(),
                    input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                    requiredVelocityMps,
                    pitchAngleRadians,
                    input.getShooterYaw(),
                    0, // spinRpm (not tracked in result)
                    input.getTargetX(), input.getTargetY(), input.getTargetZ(),
                    0 // targetRadius=0 to prevent early termination
            );

            int validCount = 0;
            for (int i = 0; i < simResult.trajectory.length; i++) {
                if (simResult.trajectory[i] == null) {
                    break;
                }
                validCount++;
            }

            // Find the point of closest 3D approach to the target so the path
            // ends near the target instead of continuing to the ground.
            double tX = input.getTargetX();
            double tY = input.getTargetY();
            double tZ = input.getTargetZ();
            int closestIndex = 0;
            double closestDist2 = Double.MAX_VALUE;
            for (int i = 0; i < validCount; i++) {
                ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion.TrajectoryState s = simResult.trajectory[i];
                if (s == null) {
                    break;
                }
                double dx = s.x - tX;
                double dy = s.y - tY;
                double dz = s.z - tZ;
                double d2 = dx * dx + dy * dy + dz * dz;
                if (d2 < closestDist2) {
                    closestDist2 = d2;
                    closestIndex = i;
                }
            }
            // Include a couple points past closest approach so the path visually
            // reaches the target area, but not the entire descent to the ground.
            int endIndex = Math.min(closestIndex + 2, validCount);

            List<Pose3d> path = new ArrayList<>();
            for (int i = 0; i < endIndex; i++) {
                ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion.TrajectoryState state = simResult.trajectory[i];
                if (state == null) {
                    break;
                }
                Translation3d translation = new Translation3d(state.x, state.y, state.z);
                Rotation3d rotation = new Rotation3d(
                        0.0,
                        Math.atan2(state.vz, Math.sqrt(state.vx * state.vx + state.vy * state.vy)),
                        Math.atan2(state.vy, state.vx)
                );
                path.add(new Pose3d(translation, rotation));
            }

            return path;
        } catch (Exception e) {
            return List.of();
        }
    }

    /**
     * Status of the trajectory solution.
     */
    public enum Status {
        SUCCESS, // Valid solution found
        OUT_OF_RANGE, // Target too far for any configuration
        VELOCITY_EXCEEDED, // Required velocity exceeds flywheel capability
        ANGLE_EXCEEDED, // Required angle outside mechanism limits
        NO_VALID_DISCRETE, // No discrete CRT solution found
        OBSTRUCTED, // Trajectory would hit obstacle (future use)
        INVALID_INPUT        // Invalid input parameters
    }

    private final Status status;
    private final String statusMessage;

    // Primary solution
    private final double pitchAngleRadians;
    private final double yawAdjustmentRadians;
    private final double requiredVelocityMps;

    // Flywheel recommendation
    private final FlywheelConfig recommendedFlywheel;
    private final FlywheelSimulator.SimulationResult flywheelSimulation;
    private final double recommendedRpm;

    // Trajectory metrics
    private final double timeOfFlightSeconds;
    private final double maxHeightMeters;
    private final double marginOfErrorMeters;

    // CRT discrete solution
    private final DiscreteShot discreteSolution;

    // Input reference
    private final ShotInput input;
    private final GamePiece gamePiece;

    // Confidence score (0-100)
    private final double confidenceScore;

    // Debug info (null unless debug mode was enabled on the solver)
    private SolveDebugInfo debugInfo;

    /**
     * Represents an RPM/angle combination that satisfies constraints.
     */
    public static class DiscreteShot {

        public final double rpmValue;
        public final double pitchAngleDegrees;
        public final int rpmTicks;        // Encoder ticks for RPM
        public final int angleTicks;      // Encoder ticks for angle
        public final double score;        // Quality score

        public DiscreteShot(double rpmValue, double pitchAngleDegrees,
                int rpmTicks, int angleTicks, double score) {
            this.rpmValue = rpmValue;
            this.pitchAngleDegrees = pitchAngleDegrees;
            this.rpmTicks = rpmTicks;
            this.angleTicks = angleTicks;
            this.score = score;
        }

        @Override
        public String toString() {
            return String.format("%.0f RPM @ %.1f° (score: %.2f)",
                    rpmValue, pitchAngleDegrees, score);
        }
    }

    /**
     * Creates a successful trajectory result.
     */
    public TrajectoryResult(ShotInput input, GamePiece gamePiece,
            double pitchAngleRadians, double yawAdjustmentRadians,
            double requiredVelocityMps,
            FlywheelConfig recommendedFlywheel,
            FlywheelSimulator.SimulationResult flywheelSimulation,
            double recommendedRpm,
            double timeOfFlightSeconds, double maxHeightMeters,
            double marginOfErrorMeters,
            DiscreteShot discreteSolution,
            double confidenceScore) {
        this.status = Status.SUCCESS;
        this.statusMessage = "Valid trajectory found";
        this.input = input;
        this.gamePiece = gamePiece;
        this.pitchAngleRadians = pitchAngleRadians;
        this.yawAdjustmentRadians = yawAdjustmentRadians;
        this.requiredVelocityMps = requiredVelocityMps;
        this.recommendedFlywheel = recommendedFlywheel;
        this.flywheelSimulation = flywheelSimulation;
        this.recommendedRpm = recommendedRpm;
        this.timeOfFlightSeconds = timeOfFlightSeconds;
        this.maxHeightMeters = maxHeightMeters;
        this.marginOfErrorMeters = marginOfErrorMeters;
        this.discreteSolution = discreteSolution;
        this.confidenceScore = confidenceScore;
    }

    /**
     * Creates a failed trajectory result.
     */
    public static TrajectoryResult failure(Status status, String message, ShotInput input) {
        return new TrajectoryResult(status, message, input);
    }

    private TrajectoryResult(Status status, String message, ShotInput input) {
        this.status = status;
        this.statusMessage = message;
        this.input = input;
        this.gamePiece = null;
        this.pitchAngleRadians = Double.NaN;
        this.yawAdjustmentRadians = Double.NaN;
        this.requiredVelocityMps = Double.NaN;
        this.recommendedFlywheel = null;
        this.flywheelSimulation = null;
        this.recommendedRpm = Double.NaN;
        this.timeOfFlightSeconds = Double.NaN;
        this.maxHeightMeters = Double.NaN;
        this.marginOfErrorMeters = Double.NaN;
        this.discreteSolution = null;
        this.confidenceScore = 0;
    }

    // Convenience methods
    /**
     * Returns true if a valid solution was found.
     */
    public boolean isSuccess() {
        return status == Status.SUCCESS;
    }

    /**
     * Gets pitch angle in degrees.
     */
    public double getPitchAngleDegrees() {
        return Math.toDegrees(pitchAngleRadians);
    }

    /**
     * Gets yaw adjustment in degrees.
     */
    public double getYawAdjustmentDegrees() {
        return Math.toDegrees(yawAdjustmentRadians);
    }

    /**
     * Gets the recommended motor power percentage (0.0 to 1.0).
     */
    public double getRecommendedMotorPowerPercent() {
        if (flywheelSimulation == null) {
            return Double.NaN;
        }
        return flywheelSimulation.motorPowerPercent;
    }

    /**
     * Gets the required exit velocity in feet per second.
     */
    public double getRequiredVelocityFps() {
        return requiredVelocityMps / 0.3048;
    }

    /**
     * Gets the wheel diameter recommendation in inches.
     */
    public double getRecommendedWheelDiameterInches() {
        if (recommendedFlywheel == null) {
            return Double.NaN;
        }
        return recommendedFlywheel.getWheelDiameterInches();
    }

    /**
     * Gets the wheel durometer recommendation.
     */
    public int getRecommendedDurometer() {
        if (recommendedFlywheel == null) {
            return 0;
        }
        return recommendedFlywheel.getMaterial().getDurometer();
    }

    /**
     * Gets maximum height in feet.
     */
    public double getMaxHeightFeet() {
        return maxHeightMeters / 0.3048;
    }

    /**
     * Gets discrete RPM if CRT solution exists.
     */
    public double getDiscreteRpm() {
        return discreteSolution != null ? discreteSolution.rpmValue : recommendedRpm;
    }

    /**
     * Gets discrete pitch angle if CRT solution exists.
     */
    public double getDiscretePitchDegrees() {
        return discreteSolution != null ? discreteSolution.pitchAngleDegrees : getPitchAngleDegrees();
    }

    // Getters
    public Status getStatus() {
        return status;
    }

    public String getStatusMessage() {
        return statusMessage;
    }

    public double getPitchAngleRadians() {
        return pitchAngleRadians;
    }

    public double getYawAdjustmentRadians() {
        return yawAdjustmentRadians;
    }

    public double getRequiredVelocityMps() {
        return requiredVelocityMps;
    }

    public FlywheelConfig getRecommendedFlywheel() {
        return recommendedFlywheel;
    }

    public FlywheelSimulator.SimulationResult getFlywheelSimulation() {
        return flywheelSimulation;
    }

    public double getRecommendedRpm() {
        return recommendedRpm;
    }

    public double getTimeOfFlightSeconds() {
        return timeOfFlightSeconds;
    }

    public double getMaxHeightMeters() {
        return maxHeightMeters;
    }

    public double getMarginOfErrorMeters() {
        return marginOfErrorMeters;
    }

    public DiscreteShot getDiscreteSolution() {
        return discreteSolution;
    }

    public ShotInput getInput() {
        return input;
    }

    public GamePiece getGamePiece() {
        return gamePiece;
    }

    public double getConfidenceScore() {
        return confidenceScore;
    }

    /**
     * Returns debug information from the solver run, or null if debug mode was
     * not enabled. Contains rejection reasons for every tested angle and all
     * candidate trajectory paths.
     *
     * @see TrajectorySolver#setDebugEnabled(boolean)
     */
    public SolveDebugInfo getDebugInfo() {
        return debugInfo;
    }

    /**
     * Attaches debug info to this result. Called internally by the solver.
     */
    void setDebugInfo(SolveDebugInfo info) {
        this.debugInfo = info;
    }

    @Override
    public String toString() {
        if (!isSuccess()) {
            return String.format("FAILED: %s - %s", status, statusMessage);
        }

        StringBuilder sb = new StringBuilder();
        sb.append("TRAJECTORY SOLUTION\n");
        sb.append(String.format("Status: %s (%.0f%% confidence)\n", status, confidenceScore));
        sb.append("\n SHOT PARAMETERS \n");
        sb.append(String.format("Pitch Angle: %.2f° (%.4f rad)\n", getPitchAngleDegrees(), pitchAngleRadians));
        sb.append(String.format("Yaw Adjustment: %.2f°\n", getYawAdjustmentDegrees()));
        sb.append(String.format("Required Velocity: %.2f m/s (%.1f fps)\n", requiredVelocityMps, getRequiredVelocityFps()));
        sb.append(String.format("Time of Flight: %.3f seconds\n", timeOfFlightSeconds));
        sb.append(String.format("Max Height: %.2f m (%.1f ft)\n", maxHeightMeters, getMaxHeightFeet()));
        sb.append(String.format("Margin of Error: %.3f m (%.1f in)\n", marginOfErrorMeters, marginOfErrorMeters / 0.0254));

        return sb.toString();
    }

    /**
     * Returns a compact single-line summary.
     */
    public String toCompactString() {
        if (!isSuccess()) {
            return String.format("FAILED: %s", statusMessage);
        }
        return String.format("%.1f° pitch, %.0f%% pwr, %.2fm/s, %.0f%% conf",
                getPitchAngleDegrees(), getRecommendedMotorPowerPercent() * 100, requiredVelocityMps,
                confidenceScore);
    }
}
