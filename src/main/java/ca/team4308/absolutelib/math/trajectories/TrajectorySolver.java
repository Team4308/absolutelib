package ca.team4308.absolutelib.math.trajectories;

import java.util.List;

import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelGenerator;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelSimulator;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;
import ca.team4308.absolutelib.math.trajectories.physics.AirResistance;
import ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion;

/**
 * Trajectory solver for FRC shooting. Handles projectile physics, flywheel
 * selection, and CRT discrete solutions.
 *
 * @see #solve(ShotInput)
 * @see #findAllCandidates(ShotInput)
 * @see #solveBestPitchDegrees(ShotInput)
 */
@SuppressWarnings("deprecation")
public class TrajectorySolver {

    /**
     * Strategy for finding a trajectory.
     */
    public enum SolveMode {
        /**
         * Two-constraint algebraic solver (default).
         */
        CONSTRAINT,
        /**
         * Full pitch-range sweep with miss-distance selection.
         */
        SWEEP
    }

    /**
     * Configuration for the solver.
     */
    public static class SolverConfig {

        private final double minPitchDegrees;
        private final double maxPitchDegrees;
        private final double minRpm;
        private final double maxRpm;

        private final double rpmTolerance;
        private final double angleTolerance;

        private final FlywheelGenerator.GenerationParams flywheelGenParams;

        private final double crtRpmResolution;
        private final double crtAngleResolution;
        private final int crtControlLoopMs;
        private final int crtEncoderTicks;

        /**
         * Multiplier for target radius when checking hits.
         */
        private final double hoopToleranceMultiplier;

        private SolverConfig(Builder builder) {
            this.minPitchDegrees = builder.minPitchDegrees;
            this.maxPitchDegrees = builder.maxPitchDegrees;
            this.minRpm = builder.minRpm;
            this.maxRpm = builder.maxRpm;
            this.rpmTolerance = builder.rpmTolerance;
            this.angleTolerance = builder.angleTolerance;
            this.flywheelGenParams = builder.flywheelGenParams;
            this.crtRpmResolution = builder.crtRpmResolution;
            this.crtAngleResolution = builder.crtAngleResolution;
            this.crtControlLoopMs = builder.crtControlLoopMs;
            this.crtEncoderTicks = builder.crtEncoderTicks;
            this.hoopToleranceMultiplier = builder.hoopToleranceMultiplier;
        }

        public double getMinPitchDegrees() {
            return minPitchDegrees;
        }

        public double getMaxPitchDegrees() {
            return maxPitchDegrees;
        }

        public double getMinRpm() {
            return minRpm;
        }

        public double getMaxRpm() {
            return maxRpm;
        }

        public double getRpmTolerance() {
            return rpmTolerance;
        }

        public double getAngleTolerance() {
            return angleTolerance;
        }

        public FlywheelGenerator.GenerationParams getFlywheelGenParams() {
            return flywheelGenParams;
        }

        public double getCrtRpmResolution() {
            return crtRpmResolution;
        }

        public double getCrtAngleResolution() {
            return crtAngleResolution;
        }

        public int getCrtControlLoopMs() {
            return crtControlLoopMs;
        }

        public int getCrtEncoderTicks() {
            return crtEncoderTicks;
        }

        public double getHoopToleranceMultiplier() {
            return hoopToleranceMultiplier;
        }

        public static Builder builder() {
            return new Builder();
        }

        public Builder toBuilder() {
            return new Builder()
                    .minPitchDegrees(minPitchDegrees)
                    .maxPitchDegrees(maxPitchDegrees)
                    .minRpm(minRpm)
                    .maxRpm(maxRpm)
                    .rpmTolerance(rpmTolerance)
                    .angleTolerance(angleTolerance)
                    .flywheelGenParams(flywheelGenParams)
                    .crtRpmResolution(crtRpmResolution)
                    .crtAngleResolution(crtAngleResolution)
                    .crtControlLoopMs(crtControlLoopMs)
                    .crtEncoderTicks(crtEncoderTicks)
                    .hoopToleranceMultiplier(hoopToleranceMultiplier);
        }

        public static class Builder {

            private double minPitchDegrees = 0;
            private double maxPitchDegrees = 90;
            private double minRpm = 0;
            private double maxRpm = 10_000;

            private double rpmTolerance = 100;
            private double angleTolerance = 1.0;

            private FlywheelGenerator.GenerationParams flywheelGenParams
                    = FlywheelGenerator.GenerationParams.defaultParams();

            private double crtRpmResolution = 1.0;
            private double crtAngleResolution = 0.1;
            private int crtControlLoopMs = 20;
            private int crtEncoderTicks = 4096;
            private double hoopToleranceMultiplier = 1.0;

            public Builder minPitchDegrees(double val) {
                this.minPitchDegrees = val;
                return this;
            }

            public Builder maxPitchDegrees(double val) {
                this.maxPitchDegrees = val;
                return this;
            }

            public Builder minRpm(double val) {
                this.minRpm = val;
                return this;
            }

            public Builder maxRpm(double val) {
                this.maxRpm = val;
                return this;
            }

            public Builder rpmTolerance(double val) {
                this.rpmTolerance = val;
                return this;
            }

            public Builder angleTolerance(double val) {
                this.angleTolerance = val;
                return this;
            }

            public Builder flywheelGenParams(FlywheelGenerator.GenerationParams val) {
                this.flywheelGenParams = val;
                return this;
            }

            public Builder crtRpmResolution(double val) {
                this.crtRpmResolution = val;
                return this;
            }

            public Builder crtAngleResolution(double val) {
                this.crtAngleResolution = val;
                return this;
            }

            public Builder crtControlLoopMs(int val) {
                this.crtControlLoopMs = val;
                return this;
            }

            public Builder crtEncoderTicks(int val) {
                this.crtEncoderTicks = val;
                return this;
            }

            public Builder hoopToleranceMultiplier(double val) {
                this.hoopToleranceMultiplier = val;
                return this;
            }

            public SolverConfig build() {
                return new SolverConfig(this);
            }
        }

        public static SolverConfig defaults() {
            return builder().build();
        }

        public static SolverConfig highAccuracy() {
            return builder()
                    .angleTolerance(0.5)
                    .rpmTolerance(50)
                    .flywheelGenParams(FlywheelGenerator.GenerationParams.detailed())
                    .crtAngleResolution(0.05)
                    .build();
        }

        public static SolverConfig quickSolve() {
            return builder()
                    .angleTolerance(2.0)
                    .rpmTolerance(200)
                    .flywheelGenParams(FlywheelGenerator.GenerationParams.quickScan())
                    .build();
        }
    }

    private final GamePiece gamePiece;
    private final SolverConfig config;
    private final ProjectileMotion projectileMotion;
    private final FlywheelGenerator flywheelGenerator;

    private FlywheelConfig cachedFlywheel;
    private boolean debugEnabled = false;
    private SolveMode solveMode = SolveMode.CONSTRAINT;

    /**
     * Sets the solve strategy.
     */
    public void setSolveMode(SolveMode mode) {
        this.solveMode = (mode != null) ? mode : SolveMode.CONSTRAINT;
    }

    /**
     * Returns the current solve strategy.
     */
    public SolveMode getSolveMode() {
        return solveMode;
    }

    /**
     * @deprecated Scoring weights are no longer used. Kept for API compat.
     */
    @Deprecated
    private ScoringWeights scoringWeights = ScoringWeights.defaults();

    /**
     * Enables or disables debug recording. Adds overhead; disable for
     * competition.
     */
    public void setDebugEnabled(boolean enabled) {
        this.debugEnabled = enabled;
    }

    /**
     * Returns whether debug mode is currently enabled.
     */
    public boolean isDebugEnabled() {
        return debugEnabled;
    }

    /**
     * @deprecated Scoring weights are no longer used.
     */
    @Deprecated
    public void setScoringWeights(ScoringWeights weights) {
        if (weights == null) {
            throw new IllegalArgumentException("weights must not be null");
        }
        this.scoringWeights = weights;
    }

    /**
     * @deprecated Scoring weights are no longer used by the solver.
     */
    @Deprecated
    public ScoringWeights getScoringWeights() {
        return scoringWeights;
    }

    private static final double ACCURACY_SCORE_MAX = 100.0;
    private static final double ACCURACY_SCORE_HIT_MIN = 50.0;
    private static final double ACCURACY_HIT_MULTIPLIER = 50.0;
    private static final double ACCURACY_MISS_BASE = 50.0;
    private static final double ACCURACY_MISS_MULTIPLIER = 25.0;

    private static final double STABILITY_OPTIMAL_ANGLE_DEG = 40.0;
    private static final double STABILITY_MAX_ANGLE_DEG = 75.0;
    private static final double STABILITY_MIN_ANGLE_DEG = 10.0;
    private static final double STABILITY_UNSTABLE_BASE = 60.0;
    private static final double STABILITY_UNSTABLE_MIN = 20.0;
    private static final double STABILITY_STABLE_BASE = 90.0;
    private static final double STABILITY_STABLE_MIN = 30.0;

    private static final double SPEED_SCORE_MAX = 95.0;
    private static final double SPEED_SCORE_RANGE = 60.0;
    private static final double SPEED_SCORE_DEFAULT = 70.0;

    private static final double CLEARANCE_SCORE_BASE = 40.0;
    private static final double CLEARANCE_SCORE_RANGE = 55.0;
    private static final double CLEARANCE_SCORE_DEFAULT = 50.0;
    private static final double CLEARANCE_MIN_HEIGHT = 0.1;

    private static final double CONFIDENCE_HIT_SCORE = 90.0;
    private static final double CONFIDENCE_MISS_BASE = 70.0;
    private static final double CONFIDENCE_MISS_MULTIPLIER = 100.0;

    private static final double ARC_LOW_MAX_DEG = 25.0;
    private static final double ARC_HIGH_MIN_DEG = 55.0;
    private static final double ARC_OPTIMAL_MIN_DEG = 40.0;
    private static final double ARC_OPTIMAL_MAX_DEG = 50.0;

    private static final double MIN_TOF_RANGE = 0.001;

    /**
     * Min pitch range (deg) after forcing high arc. If too narrow, skip the
     * force.
     */
    private static final double MIN_FORCED_ARC_RANGE_DEG = 15.0;

    /**
     * Distance (m) at which full drag compensation kicks in. Linearly
     * interpolated below this.
     */
    private static final double DRAG_COMP_FULL_RANGE_METERS = 8.0;

    /**
     * Vacuum launch velocity for a given pitch, distance, and height
     * difference.
     *
     * @return velocity in m/s, or NaN if the angle can't reach
     */
    static double calculateRequiredVelocityForPitch(double distance, double heightDiff, double pitchRad) {
        double cosTheta = Math.cos(pitchRad);
        double tanTheta = Math.tan(pitchRad);
        double denominator = distance * tanTheta - heightDiff;
        if (denominator <= 0.001) {
            return Double.NaN; // angle too shallow for this height

        }
        double g = ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.GRAVITY;
        return Math.sqrt(g * distance * distance / (2.0 * cosTheta * cosTheta * denominator));
    }

    /**
     * Two-constraint parabolic solver. Models trajectory as y = ax^2 + bx,
     * enforces (1) ball hits target center and (2) ball clears rim edge. Solves
     * for a and b, then derives pitch and velocity.
     *
     * @param d horizontal distance to target (m)
     * @param h height difference (m)
     * @param r target opening radius (m)
     * @param c rim clearance height (m)
     * @return {pitchRadians, velocityMps} or null
     */
    static double[] computeConstraintSolution(double d, double h, double r, double c) {

        if (d <= r || r <= 0 || d < 0.3) {
            return null;
        }

        double g = ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.GRAVITY;

        double denom = d * r * (d - r);
        if (Math.abs(denom) < 1e-10) {
            return null;
        }

        double a = -(h * r + c * d) / denom;

        double b = (h - a * d * d) / d;

        double theta = Math.atan(b);
        if (theta <= 0) {
            return null;
        }

        double cosTheta = Math.cos(theta);
        double v0Sq = -g / (2.0 * a * cosTheta * cosTheta);
        if (v0Sq <= 0) {
            return null;
        }

        return new double[]{theta, Math.sqrt(v0Sq)};
    }

    /**
     * Distance-scaled drag compensation. Linearly ramps from 1.0 at close range
     * to full dragCompensationMultiplier at long range.
     */
    static double calculateDragCompensation(double distance) {
        double closeRange = SolverConstants.getCloseRangeThresholdMeters();
        double fullDragComp = SolverConstants.getDragCompensationMultiplier();

        if (distance <= closeRange) {
            return 1.0;
        }
        if (distance >= DRAG_COMP_FULL_RANGE_METERS) {
            return fullDragComp;
        }

        double t = (distance - closeRange) / (DRAG_COMP_FULL_RANGE_METERS - closeRange);
        return 1.0 + t * (fullDragComp - 1.0);
    }

    /**
     * Maximum number of binary-search iterations for velocity refinement.
     */
    private static final int VELOCITY_REFINE_ITERATIONS = 8;

    /**
     * Binary-searches velocity to land the ball through the rim plane within
     * the target opening. Corrects for drag overshoot.
     *
     * @return a hit result, or null if nothing in range works
     */
    private ProjectileMotion.TrajectoryResult refineVelocityForHit(
            GamePiece gp,
            double shooterX, double shooterY, double shooterZ,
            double pitchRad, double yawRad, double spinRpm,
            double targetX, double targetY, double targetZ, double targetRadius,
            double initialVelocity) {

        double vLow = initialVelocity * 0.50;
        double vHigh = initialVelocity * 1.05;

        ProjectileMotion.TrajectoryResult bestHit = null;

        for (int i = 0; i < VELOCITY_REFINE_ITERATIONS; i++) {
            double vMid = (vLow + vHigh) / 2.0;
            ProjectileMotion.TrajectoryResult result = projectileMotion.simulate(
                    gp, shooterX, shooterY, shooterZ,
                    vMid, pitchRad, yawRad, spinRpm,
                    targetX, targetY, targetZ, targetRadius);

            if (result.hitTarget) {
                bestHit = result;
                vHigh = vMid;
            } else if (result.entryAngleDegrees >= 0) {
                vHigh = vMid;
            } else {
                vLow = vMid;
            }
        }
        return bestHit;
    }

    /**
     * Checks if a trajectory collides with any obstacle, respecting grace
     * distance and the opening exemption for descending balls.
     */
    private static boolean trajectoryCollides(ProjectileMotion.TrajectoryResult trajSim,
            ShotInput input, double shooterX, double shooterY) {
        return trajectoryCollidesInternal(trajSim, input, shooterX, shooterY, false);
    }

    /**
     * Checks collision with optional verbose logging for diagnostics.
     */
    static boolean trajectoryCollidesInternal(ProjectileMotion.TrajectoryResult trajSim,
            ShotInput input, double shooterX, double shooterY, boolean verbose) {
        if (!input.isCollisionCheckEnabled() || trajSim.trajectory.length == 0) {
            return false;
        }

        double graceDistance = SolverConstants.getCollisionGraceDistanceMeters();
        double graceDist2 = graceDistance * graceDistance;

        for (ObstacleConfig obstacle : input.getObstacles()) {
            for (int i = 0; i < trajSim.trajectory.length; i++) {
                ProjectileMotion.TrajectoryState state = trajSim.trajectory[i];
                double sdx = state.x - shooterX;
                double sdy = state.y - shooterY;
                if (sdx * sdx + sdy * sdy < graceDist2) {
                    continue;
                }

                if (state.vz < 0 && obstacle.isWithinOpening(state.x, state.y)) {
                    continue;
                }

                if (obstacle.checkCollision(state.x, state.y, state.z)) {
                    if (verbose) {
                        double distFromCenter = Math.sqrt(
                                Math.pow(state.x - obstacle.getCenterX(), 2)
                                + Math.pow(state.y - obstacle.getCenterY(), 2));
                        System.out.printf("    COLLISION at pt[%d]: (%.3f, %.3f, %.3f) vz=%.2f "
                                + "distFromCenter=%.3f opening=%.3f wallH=%.2f totalH=%.2f%n",
                                i, state.x, state.y, state.z, state.vz,
                                distFromCenter, obstacle.getOpeningDiameter() / 2.0,
                                obstacle.getWallHeight(), obstacle.getTotalHeight());
                    }
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Checks if the ball flies over the target without descending into it.
     * Exempt if the ball is descending and horizontally close enough to enter.
     */
    private static boolean isFlyover(ProjectileMotion.TrajectoryState[] trajectory,
            double targetX, double targetY, double targetZ, double targetRadius) {
        if (trajectory == null || trajectory.length == 0) {
            return false;
        }

        double bestHorizDist2 = Double.MAX_VALUE;
        double heightAtBestHoriz = 0;
        double vzAtBestHoriz = 0;

        for (ProjectileMotion.TrajectoryState st : trajectory) {
            double dx = st.x - targetX;
            double dy = st.y - targetY;
            double hd2 = dx * dx + dy * dy;
            if (hd2 < bestHorizDist2) {
                bestHorizDist2 = hd2;
                heightAtBestHoriz = st.z;
                vzAtBestHoriz = st.vz;
            }
        }

        double bestHorizDist = Math.sqrt(bestHorizDist2);

        // Only exempt descending balls that are horizontally close to the target
        if (vzAtBestHoriz < 0 && bestHorizDist <= targetRadius * SolverConstants.getHoopToleranceMultiplier()) {
            return false;
        }

        return heightAtBestHoriz > targetZ;
    }

    /**
     * Overload for findAllCandidates which uses AngleEvaluation trajectory
     * data.
     */
    private static boolean evaluationCollides(ProjectileMotion.AngleEvaluation eval,
            ShotInput input, double shooterX, double shooterY) {
        if (!input.isCollisionCheckEnabled() || eval.trajectory == null) {
            return false;
        }

        double graceDistance = SolverConstants.getCollisionGraceDistanceMeters();
        double graceDist2 = graceDistance * graceDistance;

        for (ObstacleConfig obstacle : input.getObstacles()) {
            for (ProjectileMotion.TrajectoryState state : eval.trajectory.trajectory) {
                double sdx = state.x - shooterX;
                double sdy = state.y - shooterY;
                if (sdx * sdx + sdy * sdy < graceDist2) {
                    continue;
                }

                if (state.vz < 0 && obstacle.isWithinOpening(state.x, state.y)) {
                    continue;
                }

                if (obstacle.checkCollision(state.x, state.y, state.z)) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Constraint-based core: solves the two-constraint system, validates with
     * RK4 + velocity refinement. Returns {pitch, targetX, targetY, tof} or
     * null.
     */
    private double[] solveConstraintCore(
            ShotInput input, FlywheelSimulator flywheelSimForPitch, GamePiece gp,
            double effectiveTargetX, double effectiveTargetY, double estimatedTof,
            double distance, double heightDiff, double dragComp,
            double effectiveMinPitch, double effectiveMaxPitch,
            double requiredClearance, boolean moving, SolveDebugInfo debugInfo) {

        double rimClearance = SolverConstants.getRimClearanceMeters();
        double currentClearance = rimClearance;

        double bestPitch = Double.NaN;
        ProjectileMotion.TrajectoryResult bestTraj = null;
        FlywheelSimulator.SimulationResult bestFw = null;
        double itX = effectiveTargetX, itY = effectiveTargetY;

        while (currentClearance <= rimClearance + 3.0) {
            double iterTargetX = itX;
            double iterTargetY = itY;

            if (moving && bestTraj != null && bestTraj.flightTime > 0) {
                iterTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                iterTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
            }

            double dx = iterTargetX - input.getShooterX();
            double dy = iterTargetY - input.getShooterY();
            double iterDistance = Math.sqrt(dx * dx + dy * dy);
            double requiredYaw = Math.atan2(dy, dx);

            double[] csol = computeConstraintSolution(
                    iterDistance, heightDiff, input.getTargetRadius(), currentClearance);
            if (csol == null) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(0,
                            SolveDebugInfo.RejectionReason.MISSED_TARGET,
                            Double.MAX_VALUE, 0, 0, false, new ProjectileMotion.TrajectoryState[0]);
                }
                currentClearance += 0.25;
                continue;
            }

            double pitchRad = csol[0];
            double vacuumV = csol[1];

            if (pitchRad < Math.toRadians(effectiveMinPitch)) {
                pitchRad = Math.toRadians(effectiveMinPitch);
                vacuumV = calculateRequiredVelocityForPitch(iterDistance, heightDiff, pitchRad);
                if (Double.isNaN(vacuumV) || vacuumV <= 0) {
                    currentClearance += 0.25;
                    continue;
                }
            } else if (pitchRad > Math.toRadians(effectiveMaxPitch)) {
                pitchRad = Math.toRadians(effectiveMaxPitch);
                vacuumV = calculateRequiredVelocityForPitch(iterDistance, heightDiff, pitchRad);
                if (Double.isNaN(vacuumV) || vacuumV <= 0) {
                    currentClearance += 0.25;
                    continue;
                }
            }

            double pitchDeg = Math.toDegrees(pitchRad);
            double pitchDragComp = 1.0 + (dragComp - 1.0) * Math.cos(pitchRad);
            double targetV = vacuumV * pitchDragComp;

            FlywheelSimulator.SimulationResult pitchFw
                    = flywheelSimForPitch.simulateForVelocity(targetV);
            if (!pitchFw.isAchievable) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg,
                            SolveDebugInfo.RejectionReason.ARC_TOO_LOW,
                            Double.MAX_VALUE, 0, 0, false, new ProjectileMotion.TrajectoryState[0]);
                }
                currentClearance += 0.25;
                continue;
            }

            double actualVelocity = pitchFw.exitVelocityMps;

            ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                    gp,
                    input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                    actualVelocity, pitchRad, requiredYaw,
                    pitchFw.ballSpinRpm,
                    iterTargetX, iterTargetY, input.getTargetZ(),
                    input.getTargetRadius()
            );

            if (!trajSim.hitTarget && trajSim.maxHeight > input.getTargetZ()) {
                ProjectileMotion.TrajectoryResult refined = refineVelocityForHit(
                        gp,
                        input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                        pitchRad, requiredYaw, pitchFw.ballSpinRpm,
                        iterTargetX, iterTargetY, input.getTargetZ(),
                        input.getTargetRadius(), actualVelocity);
                if (refined != null) {
                    trajSim = refined;
                    if (refined.trajectory.length > 0) {
                        ProjectileMotion.TrajectoryState s0 = refined.trajectory[0];
                        actualVelocity = Math.sqrt(s0.vx * s0.vx + s0.vy * s0.vy + s0.vz * s0.vz);
                    }
                    pitchFw = flywheelSimForPitch.simulateForVelocity(actualVelocity);
                    if (!pitchFw.isAchievable) {
                        currentClearance += 0.25;
                        continue;
                    }
                }
            }

            if (trajSim.flightTime > 0) {
                estimatedTof = trajSim.flightTime;
            }

            if (trajectoryCollides(trajSim, input, input.getShooterX(), input.getShooterY())) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.COLLISION,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                currentClearance += 0.25;
                continue;
            }
            if (requiredClearance > 0 && trajSim.maxHeight < requiredClearance) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.CLEARANCE_TOO_LOW,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                currentClearance += 0.25;
                continue;
            }

            double minArcHeight = input.getMinArcHeightMeters();
            if (minArcHeight > 0 && trajSim.maxHeight < input.getTargetZ() + minArcHeight) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.ARC_TOO_LOW,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                currentClearance += 0.25;
                continue;
            }

            double hoopTolerance = input.getTargetRadius() * config.getHoopToleranceMultiplier();
            boolean hitsTarget = trajSim.hitTarget
                    || (trajSim.descendingAtClosest && trajSim.closestApproach <= hoopTolerance
                    && trajSim.entryAngleDegrees >= SolverConstants.getMinEntryAngleDegrees());
            if (!hitsTarget) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.MISSED_TARGET,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                currentClearance += 0.25;
                continue;
            }
            if (isFlyover(trajSim.trajectory, iterTargetX, iterTargetY,
                    input.getTargetZ(), input.getTargetRadius())) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.FLYOVER,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                currentClearance += 0.25;
                continue;
            }

            bestPitch = pitchRad;
            bestTraj = trajSim;
            bestFw = pitchFw;
            itX = iterTargetX;
            itY = iterTargetY;

            double missDistance = (trajSim.horizontalDistAtCrossing >= 0)
                    ? trajSim.horizontalDistAtCrossing : trajSim.closestApproach;
            if (debugInfo != null) {
                debugInfo.recordAccepted(pitchDeg, missDistance,
                        trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime,
                        trajSim.hitTarget, trajSim.trajectory);
            }
            break;
        }

        if (Double.isNaN(bestPitch) || bestTraj == null) {
            return null;
        }

        if (moving && bestTraj.flightTime > 0) {
            for (int refine = 0; refine < 2; refine++) {
                double refTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                double refTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
                double rdx = refTargetX - input.getShooterX();
                double rdy = refTargetY - input.getShooterY();
                double refDist = Math.sqrt(rdx * rdx + rdy * rdy);
                double refYaw = Math.atan2(rdy, rdx);

                double[] refSol = computeConstraintSolution(
                        refDist, heightDiff, input.getTargetRadius(), currentClearance);
                if (refSol == null) {
                    break;
                }

                double refPitch = Math.max(Math.toRadians(effectiveMinPitch),
                        Math.min(Math.toRadians(effectiveMaxPitch), refSol[0]));
                double refVacuumV;
                if (Math.abs(refPitch - refSol[0]) > 1e-6) {
                    refVacuumV = calculateRequiredVelocityForPitch(refDist, heightDiff, refPitch);
                    if (Double.isNaN(refVacuumV) || refVacuumV <= 0) {
                        break;
                    }
                } else {
                    refVacuumV = refSol[1];
                }

                double refDragComp = 1.0 + (dragComp - 1.0) * Math.cos(refPitch);
                FlywheelSimulator.SimulationResult refFw
                        = flywheelSimForPitch.simulateForVelocity(refVacuumV * refDragComp);
                if (!refFw.isAchievable) {
                    break;
                }

                ProjectileMotion.TrajectoryResult refTraj = projectileMotion.simulate(
                        gp,
                        input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                        refFw.exitVelocityMps, refPitch, refYaw, refFw.ballSpinRpm,
                        refTargetX, refTargetY, input.getTargetZ(), input.getTargetRadius()
                );

                if (!refTraj.hitTarget && refTraj.maxHeight > input.getTargetZ()) {
                    ProjectileMotion.TrajectoryResult refined = refineVelocityForHit(
                            gp,
                            input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                            refPitch, refYaw, refFw.ballSpinRpm,
                            refTargetX, refTargetY, input.getTargetZ(),
                            input.getTargetRadius(), refFw.exitVelocityMps);
                    if (refined != null) {
                        refTraj = refined;
                        double rv = refFw.exitVelocityMps;
                        if (refined.trajectory.length > 0) {
                            ProjectileMotion.TrajectoryState s0 = refined.trajectory[0];
                            rv = Math.sqrt(s0.vx * s0.vx + s0.vy * s0.vy + s0.vz * s0.vz);
                        }
                        refFw = flywheelSimForPitch.simulateForVelocity(rv);
                        if (!refFw.isAchievable) {
                            break;
                        }
                    }
                }

                if (refTraj.flightTime > 0) {
                    estimatedTof = refTraj.flightTime;
                }
                bestPitch = refPitch;
                itX = refTargetX;
                itY = refTargetY;
            }
        }

        return new double[]{bestPitch, itX, itY, estimatedTof};
    }

    /**
     * Sweep core: tests every pitch at 0.5 deg steps, picks smallest miss
     * distance. Returns {pitch, targetX, targetY, tof} or null.
     */
    private double[] solveSweepCore(
            ShotInput input, FlywheelSimulator flywheelSimForPitch, GamePiece gp,
            double effectiveTargetX, double effectiveTargetY, double estimatedTof,
            double distance, double heightDiff, double dragComp,
            double effectiveMinPitch, double effectiveMaxPitch,
            double requiredClearance, boolean moving, SolveDebugInfo debugInfo) {

        double bestPitch = Double.NaN;
        double bestQualityScore = -1.0;
        double itX = effectiveTargetX, itY = effectiveTargetY;

        double sweepStep = 0.5;

        for (double pitchDeg = effectiveMinPitch; pitchDeg <= effectiveMaxPitch; pitchDeg += sweepStep) {
            double pitchRad = Math.toRadians(pitchDeg);

            double iterTargetX = effectiveTargetX;
            double iterTargetY = effectiveTargetY;

            if (moving && estimatedTof > 0) {
                iterTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                iterTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
            }

            double dx = iterTargetX - input.getShooterX();
            double dy = iterTargetY - input.getShooterY();
            double iterDistance = Math.sqrt(dx * dx + dy * dy);
            double requiredYaw = Math.atan2(dy, dx);

            double vacuumV = calculateRequiredVelocityForPitch(iterDistance, heightDiff, pitchRad);
            if (Double.isNaN(vacuumV) || vacuumV <= 0) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg,
                            SolveDebugInfo.RejectionReason.ARC_TOO_LOW,
                            Double.MAX_VALUE, 0, 0, false, new ProjectileMotion.TrajectoryState[0]);
                }
                continue;
            }

            double vacuumHoriz = vacuumV * Math.cos(pitchRad);
            double compensatedHoriz = vacuumHoriz * dragComp;
            double vVert = compensatedHoriz * Math.tan(pitchRad);
            double targetV = Math.sqrt(compensatedHoriz * compensatedHoriz + vVert * vVert);

            FlywheelSimulator.SimulationResult pitchFw
                    = flywheelSimForPitch.simulateForVelocity(targetV);
            if (!pitchFw.isAchievable) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg,
                            SolveDebugInfo.RejectionReason.ARC_TOO_LOW,
                            Double.MAX_VALUE, 0, 0, false, new ProjectileMotion.TrajectoryState[0]);
                }
                continue;
            }

            double actualVelocity = pitchFw.exitVelocityMps;

            ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                    gp,
                    input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                    actualVelocity, pitchRad, requiredYaw,
                    pitchFw.ballSpinRpm,
                    iterTargetX, iterTargetY, input.getTargetZ(),
                    input.getTargetRadius()
            );

            if (!trajSim.hitTarget && trajSim.maxHeight > input.getTargetZ()) {
                ProjectileMotion.TrajectoryResult refined = refineVelocityForHit(
                        gp,
                        input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                        pitchRad, requiredYaw, pitchFw.ballSpinRpm,
                        iterTargetX, iterTargetY, input.getTargetZ(),
                        input.getTargetRadius(), actualVelocity);
                if (refined != null) {
                    trajSim = refined;
                    if (refined.trajectory.length > 0) {
                        ProjectileMotion.TrajectoryState s0 = refined.trajectory[0];
                        actualVelocity = Math.sqrt(s0.vx * s0.vx + s0.vy * s0.vy + s0.vz * s0.vz);
                    }
                    pitchFw = flywheelSimForPitch.simulateForVelocity(actualVelocity);
                    if (!pitchFw.isAchievable) {
                        continue;
                    }
                }
            }

            if (trajectoryCollides(trajSim, input, input.getShooterX(), input.getShooterY())) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.COLLISION,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                continue;
            }

            if (requiredClearance > 0 && trajSim.maxHeight < requiredClearance) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.CLEARANCE_TOO_LOW,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                continue;
            }

            double minArcHeight = input.getMinArcHeightMeters();
            if (minArcHeight > 0 && trajSim.maxHeight < input.getTargetZ() + minArcHeight) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.ARC_TOO_LOW,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                continue;
            }

            double hoopTolerance = input.getTargetRadius() * config.getHoopToleranceMultiplier();
            boolean hitsTarget = trajSim.hitTarget
                    || (trajSim.descendingAtClosest && trajSim.closestApproach <= hoopTolerance
                    && trajSim.entryAngleDegrees >= SolverConstants.getMinEntryAngleDegrees());
            if (!hitsTarget) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.MISSED_TARGET,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                continue;
            }

            if (isFlyover(trajSim.trajectory, iterTargetX, iterTargetY,
                    input.getTargetZ(), input.getTargetRadius())) {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg, SolveDebugInfo.RejectionReason.FLYOVER,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime, trajSim.hitTarget, trajSim.trajectory);
                }
                continue;
            }

            double missDistance = (trajSim.horizontalDistAtCrossing >= 0)
                    ? trajSim.horizontalDistAtCrossing : trajSim.closestApproach;

            double qualityScore = computeSweepQualityScore(
                    pitchDeg, missDistance, input.getTargetRadius(),
                    trajSim.flightTime, trajSim.entryAngleDegrees);

            if (qualityScore > bestQualityScore) {
                bestQualityScore = qualityScore;
                bestPitch = pitchRad;
                itX = iterTargetX;
                itY = iterTargetY;
                if (trajSim.flightTime > 0) {
                    estimatedTof = trajSim.flightTime;
                }

                if (debugInfo != null) {
                    debugInfo.recordAccepted(pitchDeg, missDistance,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime,
                            trajSim.hitTarget, trajSim.trajectory);
                }
            } else {
                if (debugInfo != null) {
                    debugInfo.recordRejected(pitchDeg,
                            SolveDebugInfo.RejectionReason.MISSED_TARGET,
                            trajSim.closestApproach, trajSim.maxHeight, trajSim.flightTime,
                            trajSim.hitTarget, trajSim.trajectory);
                }
            }
        }

        if (Double.isNaN(bestPitch)) {
            return null;
        }

        if (moving && estimatedTof > 0) {
            for (int refine = 0; refine < 2; refine++) {
                double refTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                double refTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
                double rdx = refTargetX - input.getShooterX();
                double rdy = refTargetY - input.getShooterY();
                double refDist = Math.sqrt(rdx * rdx + rdy * rdy);
                double refYaw = Math.atan2(rdy, rdx);

                double refVacuumV = calculateRequiredVelocityForPitch(refDist, heightDiff, bestPitch);
                if (Double.isNaN(refVacuumV) || refVacuumV <= 0) {
                    break;
                }

                double refHoriz = refVacuumV * Math.cos(bestPitch);
                double refCompHoriz = refHoriz * dragComp;
                double refVert = refCompHoriz * Math.tan(bestPitch);
                double refTargetV = Math.sqrt(refCompHoriz * refCompHoriz + refVert * refVert);
                FlywheelSimulator.SimulationResult refFw
                        = flywheelSimForPitch.simulateForVelocity(refTargetV);
                if (!refFw.isAchievable) {
                    break;
                }

                ProjectileMotion.TrajectoryResult refTraj = projectileMotion.simulate(
                        gp,
                        input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                        refFw.exitVelocityMps, bestPitch, refYaw, refFw.ballSpinRpm,
                        refTargetX, refTargetY, input.getTargetZ(), input.getTargetRadius()
                );

                if (!refTraj.hitTarget && refTraj.maxHeight > input.getTargetZ()) {
                    ProjectileMotion.TrajectoryResult refined = refineVelocityForHit(
                            gp,
                            input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                            bestPitch, refYaw, refFw.ballSpinRpm,
                            refTargetX, refTargetY, input.getTargetZ(),
                            input.getTargetRadius(), refFw.exitVelocityMps);
                    if (refined != null) {
                        refTraj = refined;
                        double rv = refFw.exitVelocityMps;
                        if (refined.trajectory.length > 0) {
                            ProjectileMotion.TrajectoryState s0 = refined.trajectory[0];
                            rv = Math.sqrt(s0.vx * s0.vx + s0.vy * s0.vy + s0.vz * s0.vz);
                        }
                        refFw = flywheelSimForPitch.simulateForVelocity(rv);
                        if (!refFw.isAchievable) {
                            break;
                        }
                    }
                }

                if (refTraj.flightTime > 0) {
                    estimatedTof = refTraj.flightTime;
                }
                itX = refTargetX;
                itY = refTargetY;
            }
        }

        return new double[]{bestPitch, itX, itY, estimatedTof};
    }

    /**
     * Computes a composite quality score for a SWEEP candidate. Higher scores
     * indicate better overall trajectory quality.
     *
     * <p>
     * Score components:
     * <ul>
     * <li>Accuracy (40%): how close to target center (miss distance / target
     * radius)</li>
     * <li>Stability (30%): deviation from optimal ~45° pitch, steep angle
     * penalty</li>
     * <li>Speed (20%): lower time-of-flight is better</li>
     * <li>Entry angle (10%): steeper entry drops into hub better</li>
     * </ul>
     *
     * @param pitchDeg candidate pitch in degrees
     * @param missDistance miss distance in meters
     * @param targetRadius target acceptance radius in meters
     * @param timeOfFlight simulated time of flight in seconds
     * @param entryAngleDeg entry angle into target in degrees
     * @return quality score in [0, 100]
     */
    private double computeSweepQualityScore(double pitchDeg, double missDistance,
            double targetRadius, double timeOfFlight,
            double entryAngleDeg) {
        double accuracyScore;
        if (targetRadius > 0) {
            double relMiss = missDistance / targetRadius;
            accuracyScore = Math.max(0, 40.0 * (1.0 - relMiss));
        } else {
            accuracyScore = missDistance < 0.01 ? 40.0 : 0.0;
        }

        double optimalPitch = 45.0;
        double deviation = Math.abs(pitchDeg - optimalPitch);
        double stabilityScore = Math.max(0, 30.0 * (1.0 - deviation / 45.0));
        if (pitchDeg > 70.0) {
            stabilityScore *= 0.5;
        }

        double speedScore = Math.max(0, 20.0 * (1.0 - timeOfFlight / 3.0));

        double entryScore = Math.min(10.0, entryAngleDeg / 9.0);

        return accuracyScore + stabilityScore + speedScore + entryScore;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {

        private GamePiece gamePiece = GamePieces.getCurrent();
        private SolverConfig config = SolverConfig.defaults();

        public Builder gamePiece(GamePiece val) {
            this.gamePiece = val;
            return this;
        }

        public Builder config(SolverConfig val) {
            this.config = val;
            return this;
        }

        public TrajectorySolver build() {
            return new TrajectorySolver(gamePiece, config);
        }
    }

    /**
     * Creates a solver for the given game piece.
     */
    public TrajectorySolver(GamePiece gamePiece) {
        this(gamePiece, SolverConfig.defaults());
    }

    /**
     * Creates a solver with custom configuration.
     */
    public TrajectorySolver(GamePiece gamePiece, SolverConfig config) {
        this.gamePiece = gamePiece;
        this.config = config;

        AirResistance airResistance = new AirResistance(true, 1.18,
                ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.FOAM_BALL_DRAG_COEFFICIENT, true);
        this.projectileMotion = new ProjectileMotion(airResistance);
        this.flywheelGenerator = new FlywheelGenerator(gamePiece, config.getFlywheelGenParams());
    }

    /**
     * Creates a solver optimized for 2026 REBUILT game.
     *
     * @return A new TrajectorySolver configured for 2026 REBUILT
     */
    public static TrajectorySolver forGame2026() {
        return forGamePiece(GamePieces.REBUILT_2026_BALL);
    }

    /**
     * Creates a solver for a specific game year.
     *
     * @param year The FRC game year
     * @return A new TrajectorySolver configured for that year's game piece
     */
    public static TrajectorySolver forYear(int year) {
        GamePiece piece = GamePieces.getByYear(year);
        if (piece == null) {
            piece = GamePieces.getCurrent();
        }
        return forGamePiece(piece);
    }

    /**
     * Creates a solver for a specific game piece.
     *
     * @param gamePiece The game piece to use
     * @return A new TrajectorySolver configured for that game piece
     */
    public static TrajectorySolver forGamePiece(GamePiece gamePiece) {
        return new TrajectorySolver(gamePiece);
    }

    /**
     * Solves for the trajectory. Main entry point.
     *
     * @param input shot parameters
     * @return result with recommended pitch, RPM, etc.
     */
    public TrajectoryResult solve(ShotInput input) {
        if (input == null) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.INVALID_INPUT,
                    "Shot input cannot be null",
                    input
            );
        }

        boolean moving = Math.abs(input.getRobotVx()) > SolverConstants.getMovementThresholdMps()
                || Math.abs(input.getRobotVy()) > SolverConstants.getMovementThresholdMps();
        int convergenceIterations = moving
                ? SolverConstants.getMovingConvergenceIterations()
                : SolverConstants.getStationaryIterations();

        double distance = input.getHorizontalDistanceMeters();
        double estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();

        double effectiveTargetX = input.getTargetX();
        double effectiveTargetY = input.getTargetY();

        for (int i = 0; i < convergenceIterations; i++) {
            if (moving) {
                effectiveTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                effectiveTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
                double dx = effectiveTargetX - input.getShooterX();
                double dy = effectiveTargetY - input.getShooterY();
                distance = Math.sqrt(dx * dx + dy * dy);
                estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
            }
        }

        double heightDiff = input.getHeightDifferenceMeters();

        if (distance < SolverConstants.getMinTargetDistanceMeters()) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.INVALID_INPUT,
                    String.format("Target too close (< %.2fm)", SolverConstants.getMinTargetDistanceMeters()),
                    input
            );
        }

        boolean pathCrossesObstacle = input.pathRequiresArc();
        double requiredClearance = input.getRequiredClearanceHeight();

        double effectiveMinPitch = input.getMinPitchDegrees();
        double effectiveMaxPitch = input.getMaxPitchDegrees();

        if (pathCrossesObstacle) {
            double forcedMin = SolverConstants.getForceHighArcMinPitchDegrees();
            if (effectiveMaxPitch - forcedMin >= MIN_FORCED_ARC_RANGE_DEG) {
                effectiveMinPitch = Math.max(effectiveMinPitch, forcedMin);
            }
        }

        double dragComp = calculateDragCompensation(distance);

        double minVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff);
        boolean isCloseRange = distance < SolverConstants.getCloseRangeThresholdMeters();
        double velocityBuffer = isCloseRange
                ? SolverConstants.getCloseRangeVelocityMultiplier()
                : SolverConstants.getVelocityBufferMultiplier() * dragComp;
        double representativeVelocity = minVelocity * velocityBuffer;

        double maxPitchV = calculateRequiredVelocityForPitch(distance, heightDiff,
                Math.toRadians(effectiveMaxPitch));
        if (!Double.isNaN(maxPitchV) && maxPitchV > 0) {
            representativeVelocity = Math.max(representativeVelocity, maxPitchV * dragComp);
        }

        FlywheelGenerator.GenerationResult genResult;
        if (cachedFlywheel != null) {
            FlywheelSimulator simulator = new FlywheelSimulator(cachedFlywheel, gamePiece);
            FlywheelSimulator.SimulationResult simResult = simulator.simulateForVelocity(representativeVelocity);

            if (simResult.isAchievable) {
                genResult = new FlywheelGenerator.GenerationResult(
                        List.of(new FlywheelGenerator.ScoredConfig(
                                cachedFlywheel, simResult,
                                simulator.scoreConfiguration(representativeVelocity)
                        )), 1
                );
            } else {
                cachedFlywheel = null;
                genResult = flywheelGenerator.generateAndEvaluate(representativeVelocity);
            }
        } else {
            genResult = flywheelGenerator.generateAndEvaluate(representativeVelocity);
        }

        if (genResult.achievableCount == 0) {
            genResult = flywheelGenerator.evaluatePresets(representativeVelocity);
        }

        double baseVelocity = minVelocity * velocityBuffer;
        if (genResult.achievableCount == 0 && representativeVelocity > baseVelocity) {
            genResult = flywheelGenerator.generateAndEvaluate(baseVelocity);
            if (genResult.achievableCount == 0) {
                genResult = flywheelGenerator.evaluatePresets(baseVelocity);
            }
        }

        if (genResult.achievableCount == 0) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.VELOCITY_EXCEEDED,
                    String.format("No flywheel can achieve required velocity: %.2f m/s", baseVelocity),
                    input
            );
        }

        FlywheelGenerator.ScoredConfig bestFlywheel = genResult.bestConfig;
        FlywheelConfig flywheel = bestFlywheel.config;

        cachedFlywheel = flywheel;

        FlywheelSimulator flywheelSimForPitch = new FlywheelSimulator(flywheel, gamePiece);

        double bestPitchAngle = Double.NaN;
        ProjectileMotion.TrajectoryResult bestTrajSim = null;
        FlywheelSimulator.SimulationResult bestFlywheelSim = null;

        SolveDebugInfo debugInfo = debugEnabled ? new SolveDebugInfo() : null;

        double[] coreResult;

        if (solveMode == SolveMode.SWEEP) {
            coreResult = solveSweepCore(input, flywheelSimForPitch, gamePiece,
                    effectiveTargetX, effectiveTargetY, estimatedTof,
                    distance, heightDiff, dragComp,
                    effectiveMinPitch, effectiveMaxPitch,
                    requiredClearance, moving, debugInfo);
        } else {

            coreResult = solveConstraintCore(input, flywheelSimForPitch, gamePiece,
                    effectiveTargetX, effectiveTargetY, estimatedTof,
                    distance, heightDiff, dragComp,
                    effectiveMinPitch, effectiveMaxPitch,
                    requiredClearance, moving, debugInfo);
            if (coreResult == null) {

                coreResult = solveSweepCore(input, flywheelSimForPitch, gamePiece,
                        effectiveTargetX, effectiveTargetY, estimatedTof,
                        distance, heightDiff, dragComp,
                        effectiveMinPitch, effectiveMaxPitch,
                        requiredClearance, moving, debugInfo);
            }
        }

        if (coreResult != null) {
            bestPitchAngle = coreResult[0];
            effectiveTargetX = coreResult[1];
            effectiveTargetY = coreResult[2];
            estimatedTof = coreResult[3];

            double dx2 = effectiveTargetX - input.getShooterX();
            double dy2 = effectiveTargetY - input.getShooterY();
            double bestYaw = Math.atan2(dy2, dx2);
            double bestIterDist = Math.sqrt(dx2 * dx2 + dy2 * dy2);

            double bestVacV = calculateRequiredVelocityForPitch(bestIterDist, heightDiff, bestPitchAngle);
            if (Double.isNaN(bestVacV) || bestVacV <= 0) {
                // Fall through to failure
            } else {
                double bestDragComp = 1.0 + (dragComp - 1.0) * Math.cos(bestPitchAngle);
                FlywheelSimulator.SimulationResult fw = flywheelSimForPitch.simulateForVelocity(bestVacV * bestDragComp);
                if (fw.isAchievable) {
                    ProjectileMotion.TrajectoryResult traj = projectileMotion.simulate(
                            gamePiece,
                            input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                            fw.exitVelocityMps, bestPitchAngle, bestYaw, fw.ballSpinRpm,
                            effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                            input.getTargetRadius());

                    if (!traj.hitTarget && traj.maxHeight > input.getTargetZ()) {
                        ProjectileMotion.TrajectoryResult refined = refineVelocityForHit(
                                gamePiece,
                                input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                                bestPitchAngle, bestYaw, fw.ballSpinRpm,
                                effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                                input.getTargetRadius(), fw.exitVelocityMps);
                        if (refined != null) {
                            traj = refined;
                            double refV = fw.exitVelocityMps;
                            if (refined.trajectory.length > 0) {
                                ProjectileMotion.TrajectoryState s0 = refined.trajectory[0];
                                refV = Math.sqrt(s0.vx * s0.vx + s0.vy * s0.vy + s0.vz * s0.vz);
                            }
                            fw = flywheelSimForPitch.simulateForVelocity(refV);
                        }
                    }

                    if (fw.isAchievable) {
                        bestTrajSim = traj;
                        bestFlywheelSim = fw;
                    }
                }
            }
        }

        if (Double.isNaN(bestPitchAngle) || bestTrajSim == null) {
            String reason = pathCrossesObstacle
                    ? "No collision-free trajectory found; all angles hit obstacles or miss target"
                    : "No trajectory solution exists for given distance and velocity";
            TrajectoryResult failResult = TrajectoryResult.failure(
                    TrajectoryResult.Status.OUT_OF_RANGE,
                    reason,
                    input
            );
            if (debugInfo != null) {
                failResult.setDebugInfo(debugInfo);
            }
            return failResult;
        }

        double pitchDegrees = Math.toDegrees(bestPitchAngle);

        double dx = effectiveTargetX - input.getShooterX();
        double dy = effectiveTargetY - input.getShooterY();
        double requiredYaw = Math.atan2(dy, dx);
        double yawAdjustment = requiredYaw - input.getShooterYaw();
        while (yawAdjustment > Math.PI) {
            yawAdjustment -= 2 * Math.PI;
        }
        while (yawAdjustment < -Math.PI) {
            yawAdjustment += 2 * Math.PI;
        }

        double requiredRpm = bestFlywheelSim.requiredWheelRpm;
        double actualVelocity = bestFlywheelSim.exitVelocityMps;
        double timeOfFlight = bestTrajSim.flightTime;
        double maxHeight = bestTrajSim.maxHeight;
        double marginOfError = bestTrajSim.closestApproach;

        TrajectoryResult.DiscreteShot discreteSolution = new TrajectoryResult.DiscreteShot(
                requiredRpm, pitchDegrees,
                (int) (requiredRpm / config.getCrtRpmResolution()),
                (int) (pitchDegrees / config.getCrtAngleResolution()),
                50.0
        );

        double confidence = calculateConfidence(
                bestFlywheel.score,
                bestTrajSim.hitTarget,
                marginOfError,
                input.getTargetRadius(),
                discreteSolution.score
        );

        TrajectoryResult successResult = new TrajectoryResult(
                input, gamePiece,
                bestPitchAngle, yawAdjustment, actualVelocity,
                flywheel, bestFlywheelSim, requiredRpm,
                timeOfFlight, maxHeight, marginOfError,
                discreteSolution, confidence
        );
        if (debugInfo != null) {
            successResult.setDebugInfo(debugInfo);
        }
        return successResult;
    }

    /**
     * Solves for multiple velocities to show trajectory options. Generates
     * solutions across a range of possible velocities.
     *
     * @param input Shot input
     * @param velocitySteps Number of velocity steps to try
     * @return Array of trajectory results at different velocities
     */
    public TrajectoryResult[] solveRange(ShotInput input, int velocitySteps) {
        double distance = input.getHorizontalDistanceMeters();
        double heightDiff = input.getHeightDifferenceMeters();

        double minVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff)
                * calculateDragCompensation(distance);
        double maxVelocity = minVelocity * SolverConstants.getMaxVelocityRangeMultiplier();

        TrajectoryResult[] results = new TrajectoryResult[velocitySteps];

        FlywheelGenerator.GenerationResult genResult
                = flywheelGenerator.generateForVelocityRange(
                        minVelocity * (1.0 / SolverConstants.getMinVelocityRangeMultiplier()),
                        maxVelocity * SolverConstants.getMinVelocityRangeMultiplier());

        for (int i = 0; i < velocitySteps; i++) {
            double targetVelocity = minVelocity + (maxVelocity - minVelocity) * i / (velocitySteps - 1);

            FlywheelConfig bestFlywheel = null;
            double bestScore = -1;

            for (FlywheelGenerator.ScoredConfig scored : genResult.configurations) {
                FlywheelSimulator sim = new FlywheelSimulator(scored.config, gamePiece);
                FlywheelSimulator.SimulationResult simResult = sim.simulateForVelocity(targetVelocity);
                if (simResult.isAchievable && scored.score > bestScore) {
                    bestFlywheel = scored.config;
                    bestScore = scored.score;
                }
            }

            if (bestFlywheel != null) {
                results[i] = solveWithFlywheel(input, bestFlywheel);
            } else {
                results[i] = solve(input);
            }
        }

        return results;
    }

    /**
     * Solves using a specific flywheel configuration.
     */
    public TrajectoryResult solveWithFlywheel(ShotInput input, FlywheelConfig flywheel) {
        FlywheelConfig previousCache = cachedFlywheel;
        cachedFlywheel = flywheel;

        try {
            return solve(input);
        } finally {
            cachedFlywheel = previousCache;
        }
    }

    /**
     * Evaluates an existing configuration against a shot.
     */
    public TrajectoryResult evaluate(ShotInput input, FlywheelConfig flywheel, double rpm, double pitchDegrees) {
        FlywheelSimulator simulator = new FlywheelSimulator(flywheel, gamePiece);
        FlywheelSimulator.SimulationResult simResult = simulator.simulateAtRpm(rpm);

        if (!simResult.isAchievable) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.VELOCITY_EXCEEDED,
                    "Specified RPM not achievable with this flywheel",
                    input
            );
        }

        double pitchRadians = Math.toRadians(pitchDegrees);
        double velocity = simResult.exitVelocityMps;

        ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                gamePiece,
                input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                velocity, pitchRadians, input.getRequiredYawRadians(),
                simResult.ballSpinRpm,
                input.getTargetX(), input.getTargetY(), input.getTargetZ(),
                input.getTargetRadius()
        );

        double confidence = trajSim.hitTarget ? CONFIDENCE_HIT_SCORE
                : Math.max(0, CONFIDENCE_MISS_BASE - trajSim.closestApproach * CONFIDENCE_MISS_MULTIPLIER);

        TrajectoryResult.DiscreteShot discreteSolution
                = new TrajectoryResult.DiscreteShot(
                        rpm, pitchDegrees,
                        (int) (rpm / config.getCrtRpmResolution()),
                        (int) (pitchDegrees / config.getCrtAngleResolution()),
                        confidence / 2
                );

        return new TrajectoryResult(
                input, gamePiece,
                pitchRadians, input.getYawAdjustmentRadians(), velocity,
                flywheel, simResult, rpm,
                trajSim.flightTime, trajSim.maxHeight, trajSim.closestApproach,
                discreteSolution, confidence
        );
    }

    /**
     * Finds the best pitch at a given flywheel RPM (e.g. for rapid-fire when
     * the flywheel is slower than ideal).
     *
     * @param input shot parameters
     * @param flywheel flywheel config to simulate
     * @param currentRpm measured RPM from encoder
     * @return result at the given RPM, or failure if unreachable
     */
    public TrajectoryResult solveAtCurrentRpm(ShotInput input, FlywheelConfig flywheel, double currentRpm) {
        if (input == null) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.INVALID_INPUT,
                    "Shot input cannot be null",
                    input
            );
        }

        FlywheelSimulator simulator = new FlywheelSimulator(flywheel, gamePiece);
        FlywheelSimulator.SimulationResult simResult = simulator.simulateAtRpm(currentRpm);

        if (!simResult.isAchievable) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.VELOCITY_EXCEEDED,
                    String.format("Current RPM (%.0f) not achievable with this flywheel", currentRpm),
                    input
            );
        }

        double actualVelocity = simResult.exitVelocityMps;
        double ballSpin = simResult.ballSpinRpm;

        boolean moving = Math.abs(input.getRobotVx()) > SolverConstants.getMovementThresholdMps()
                || Math.abs(input.getRobotVy()) > SolverConstants.getMovementThresholdMps();
        int convergenceIterations = moving
                ? SolverConstants.getMovingConvergenceIterations()
                : SolverConstants.getStationaryIterations();

        double distance = input.getHorizontalDistanceMeters();
        double estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();

        double effectiveTargetX = input.getTargetX();
        double effectiveTargetY = input.getTargetY();

        for (int i = 0; i < convergenceIterations; i++) {
            if (moving) {
                effectiveTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                effectiveTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
                double ddx = effectiveTargetX - input.getShooterX();
                double ddy = effectiveTargetY - input.getShooterY();
                distance = Math.sqrt(ddx * ddx + ddy * ddy);
                estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
            }
        }

        if (distance < SolverConstants.getMinTargetDistanceMeters()) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.INVALID_INPUT,
                    String.format("Target too close (< %.2fm)", SolverConstants.getMinTargetDistanceMeters()),
                    input
            );
        }

        boolean pathCrossesObstacle = input.pathRequiresArc();
        double requiredClearance = input.getRequiredClearanceHeight();

        double effectiveMinPitch = input.getMinPitchDegrees();
        double effectiveMaxPitch = input.getMaxPitchDegrees();

        if (pathCrossesObstacle) {
            double forcedMin = SolverConstants.getForceHighArcMinPitchDegrees();
            if (effectiveMaxPitch - forcedMin >= MIN_FORCED_ARC_RANGE_DEG) {
                effectiveMinPitch = Math.max(effectiveMinPitch, forcedMin);
            }
        }

        double bestPitchAngle = Double.NaN;
        ProjectileMotion.TrajectoryResult bestTrajSim = null;

        double g = ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants.GRAVITY;
        double v = actualVelocity;
        double d = distance;
        double h = input.getHeightDifferenceMeters();
        double disc = v * v * v * v - g * (g * d * d + 2.0 * h * v * v);

        if (disc >= 0) {
            double pitchRad = Math.atan((v * v + Math.sqrt(disc)) / (g * d));

            pitchRad = Math.max(Math.toRadians(effectiveMinPitch),
                    Math.min(Math.toRadians(effectiveMaxPitch), pitchRad));

            double dx = effectiveTargetX - input.getShooterX();
            double dy = effectiveTargetY - input.getShooterY();
            double requiredYaw = Math.atan2(dy, dx);

            ProjectileMotion.TrajectoryResult trajSim = projectileMotion.simulate(
                    gamePiece,
                    input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                    actualVelocity, pitchRad, requiredYaw, ballSpin,
                    effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                    input.getTargetRadius()
            );

            if (trajSim.flightTime > 0) {
                estimatedTof = trajSim.flightTime;
            }

            boolean valid = true;
            if (trajectoryCollides(trajSim, input, input.getShooterX(), input.getShooterY())) {
                valid = false;
            }
            if (valid && requiredClearance > 0 && trajSim.maxHeight < requiredClearance) {
                valid = false;
            }
            double minArcHeight = input.getMinArcHeightMeters();
            if (valid && minArcHeight > 0 && trajSim.maxHeight < input.getTargetZ() + minArcHeight) {
                valid = false;
            }

            double hoopTolerance = input.getTargetRadius() * config.getHoopToleranceMultiplier();
            boolean hitsTarget = trajSim.hitTarget
                    || (trajSim.descendingAtClosest && trajSim.closestApproach <= hoopTolerance
                    && trajSim.entryAngleDegrees >= SolverConstants.getMinEntryAngleDegrees());
            if (valid && !hitsTarget) {
                valid = false;
            }
            if (valid && isFlyover(trajSim.trajectory, effectiveTargetX, effectiveTargetY,
                    input.getTargetZ(), input.getTargetRadius())) {
                valid = false;
            }

            if (valid) {
                bestPitchAngle = pitchRad;
                bestTrajSim = trajSim;
            }
        }

        if (Double.isNaN(bestPitchAngle) || bestTrajSim == null) {
            return TrajectoryResult.failure(
                    TrajectoryResult.Status.OUT_OF_RANGE,
                    String.format("No trajectory found at current RPM (%.0f, exit velocity %.2f m/s)",
                            currentRpm, actualVelocity),
                    input
            );
        }

        double pitchDegrees = Math.toDegrees(bestPitchAngle);

        double dx = effectiveTargetX - input.getShooterX();
        double dy = effectiveTargetY - input.getShooterY();
        double requiredYaw = Math.atan2(dy, dx);
        double yawAdjustment = requiredYaw - input.getShooterYaw();
        while (yawAdjustment > Math.PI) {
            yawAdjustment -= 2 * Math.PI;
        }
        while (yawAdjustment < -Math.PI) {
            yawAdjustment += 2 * Math.PI;
        }

        TrajectoryResult.DiscreteShot discreteSolution = new TrajectoryResult.DiscreteShot(
                currentRpm, pitchDegrees,
                (int) (currentRpm / config.getCrtRpmResolution()),
                (int) (pitchDegrees / config.getCrtAngleResolution()),
                50.0
        );

        double confidence = calculateConfidence(
                simulator.scoreConfiguration(actualVelocity),
                bestTrajSim.hitTarget,
                bestTrajSim.closestApproach,
                input.getTargetRadius(),
                discreteSolution.score
        );

        return new TrajectoryResult(
                input, gamePiece,
                bestPitchAngle, yawAdjustment, actualVelocity,
                flywheel, simResult, currentRpm,
                bestTrajSim.flightTime, bestTrajSim.maxHeight, bestTrajSim.closestApproach,
                discreteSolution, confidence
        );
    }

    /**
     * Finds all shot candidates sorted by confidence.
     *
     * @param input shot parameters
     * @return candidates list, possibly empty
     */
    public ShotCandidateList findAllCandidates(ShotInput input) {
        if (input == null) {
            return ShotCandidateList.empty(input, "Shot input cannot be null");
        }

        double distance = input.getHorizontalDistanceMeters();
        if (distance < SolverConstants.getMinTargetDistanceMeters()) {
            return ShotCandidateList.empty(input,
                    String.format("Target too close (< %.2fm)", SolverConstants.getMinTargetDistanceMeters()));
        }

        boolean moving = Math.abs(input.getRobotVx()) > SolverConstants.getMovementThresholdMps()
                || Math.abs(input.getRobotVy()) > SolverConstants.getMovementThresholdMps();
        double estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();

        double effectiveTargetX = input.getTargetX();
        double effectiveTargetY = input.getTargetY();

        if (moving) {
            int convergenceIters = SolverConstants.getMovingConvergenceIterations();
            for (int i = 0; i < convergenceIters; i++) {
                effectiveTargetX = input.getTargetX() - input.getRobotVx() * estimatedTof;
                effectiveTargetY = input.getTargetY() - input.getRobotVy() * estimatedTof;
                double dx = effectiveTargetX - input.getShooterX();
                double dy = effectiveTargetY - input.getShooterY();
                distance = Math.sqrt(dx * dx + dy * dy);
                estimatedTof = distance / SolverConstants.getInitialVelocityEstimateMps();
            }
        }

        double heightDiff = input.getHeightDifferenceMeters();
        double yawAngle = Math.atan2(effectiveTargetY - input.getShooterY(),
                effectiveTargetX - input.getShooterX());

        boolean pathCrossesObstacle = input.pathRequiresArc();
        double requiredClearance = input.getRequiredClearanceHeight();

        double effectiveMinPitch = input.getMinPitchDegrees();
        double effectiveMaxPitch = input.getMaxPitchDegrees();

        if (pathCrossesObstacle) {
            double forcedMin = SolverConstants.getForceHighArcMinPitchDegrees();
            if (effectiveMaxPitch - forcedMin >= MIN_FORCED_ARC_RANGE_DEG) {
                effectiveMinPitch = Math.max(effectiveMinPitch, forcedMin);
            }
        }

        double minRequiredVelocity = projectileMotion.calculateMinimumVelocity(distance, heightDiff);
        double dragComp = calculateDragCompensation(distance);
        double effectiveMinVelocity = Math.max(input.getMinVelocityMps(),
                minRequiredVelocity * SolverConstants.getMinVelocityRangeMultiplier() * dragComp);
        double effectiveMaxVelocity = input.getMaxVelocityMps();

        if (effectiveMinVelocity > effectiveMaxVelocity) {
            return ShotCandidateList.empty(input,
                    String.format("Required velocity %.1f m/s exceeds maximum %.1f m/s",
                            effectiveMinVelocity, effectiveMaxVelocity));
        }

        ProjectileMotion.AngleEvaluation[] evaluations = projectileMotion.findAllAnglesWithVelocity(
                gamePiece,
                input.getShooterX(), input.getShooterY(), input.getShooterZ(),
                effectiveTargetX, effectiveTargetY, input.getTargetZ(),
                input.getTargetRadius(), 0, // No spin for initial calculation
                effectiveMinPitch, effectiveMaxPitch, input.getAngleStepDegrees(),
                effectiveMinVelocity, effectiveMaxVelocity
        );

        if (evaluations.length == 0) {
            return ShotCandidateList.empty(input, "No valid trajectories found in angle/velocity range");
        }

        java.util.List<ShotCandidate> candidates = new java.util.ArrayList<>();

        double maxTof = 0, minTof = Double.MAX_VALUE;
        double maxHeight = 0;

        for (ProjectileMotion.AngleEvaluation eval : evaluations) {
            if (evaluationCollides(eval, input, input.getShooterX(), input.getShooterY())) {
                continue;
            }

            if (requiredClearance > 0 && eval.maxHeight < requiredClearance) {
                continue;
            }

            if (eval.trajectory != null && isFlyover(eval.trajectory.trajectory,
                    effectiveTargetX, effectiveTargetY, input.getTargetZ(), input.getTargetRadius())) {
                continue;
            }

            if (eval.timeOfFlight > maxTof) {
                maxTof = eval.timeOfFlight;
            }
            if (eval.timeOfFlight < minTof && eval.timeOfFlight > 0) {
                minTof = eval.timeOfFlight;
            }
            if (eval.maxHeight > maxHeight) {
                maxHeight = eval.maxHeight;
            }
        }

        double tofRange = maxTof - minTof;
        if (tofRange < MIN_TOF_RANGE) {
            tofRange = 1;
        }

        for (ProjectileMotion.AngleEvaluation eval : evaluations) {
            if (evaluationCollides(eval, input, input.getShooterX(), input.getShooterY())) {
                continue;
            }

            if (requiredClearance > 0 && eval.maxHeight < requiredClearance) {
                continue;
            }

            if (eval.trajectory != null && isFlyover(eval.trajectory.trajectory,
                    effectiveTargetX, effectiveTargetY, input.getTargetZ(), input.getTargetRadius())) {
                continue;
            }

            ShotCandidate candidate = buildCandidate(eval, input, yawAngle, minTof, tofRange, maxHeight);
            candidates.add(candidate);
        }

        if (candidates.size() > input.getMaxCandidates()) {
            java.util.Collections.sort(candidates);
            candidates = candidates.subList(0, input.getMaxCandidates());
        }

        if (candidates.isEmpty()) {
            String reason = pathCrossesObstacle
                    ? "All trajectories collide with obstacles or fail clearance"
                    : "No valid trajectories found in angle/velocity range";
            return ShotCandidateList.empty(input, reason);
        }

        return new ShotCandidateList(candidates, input);
    }

    /**
     * Builds a ShotCandidate from an angle evaluation with proper scoring.
     */
    private ShotCandidate buildCandidate(ProjectileMotion.AngleEvaluation eval, ShotInput input,
            double yawAngle, double minTof, double tofRange, double maxHeight) {
        // Calculate scores
        double accuracyScore = calculateAccuracyScore(eval, input.getTargetRadius());
        double stabilityScore = calculateStabilityScore(eval);
        double speedScore = calculateSpeedScore(eval, minTof, tofRange);
        double clearanceScore = calculateClearanceScore(eval, maxHeight);

        // Determine arc type
        ShotCandidate.ArcType arcType = determineArcType(eval.getPitchDegrees());

        return ShotCandidate.builder()
                .pitchAngleRadians(eval.pitchRadians)
                .requiredVelocityMps(eval.velocity)
                .yawAngleRadians(yawAngle)
                .timeOfFlightSeconds(eval.timeOfFlight)
                .maxHeightMeters(eval.maxHeight)
                .closestApproachMeters(eval.closestApproach)
                .hitsTarget(eval.hitsTarget)
                .arcType(arcType)
                .accuracyScore(accuracyScore)
                .stabilityScore(stabilityScore)
                .speedScore(speedScore)
                .clearanceScore(clearanceScore)
                .build();
    }

    /**
     * Calculates accuracy score based on how close trajectory comes to target
     * center.
     */
    private double calculateAccuracyScore(ProjectileMotion.AngleEvaluation eval, double targetRadius) {
        if (eval.hitsTarget) {
            double relativeApproach = eval.closestApproach / targetRadius;
            return Math.max(ACCURACY_SCORE_HIT_MIN,
                    ACCURACY_SCORE_MAX - relativeApproach * ACCURACY_HIT_MULTIPLIER);
        } else {
            double relativeError = eval.closestApproach / targetRadius;
            return Math.max(0, ACCURACY_MISS_BASE
                    - (relativeError - 1) * ACCURACY_MISS_MULTIPLIER);
        }
    }

    /**
     * Calculates stability score based on angle sensitivity. Mid-range angles
     * (~30-50°) are typically more stable.
     */
    private double calculateStabilityScore(ProjectileMotion.AngleEvaluation eval) {
        double angleDeg = eval.getPitchDegrees();

        double optimalAngle = STABILITY_OPTIMAL_ANGLE_DEG;
        double deviation = Math.abs(angleDeg - optimalAngle);

        if (angleDeg > STABILITY_MAX_ANGLE_DEG
                || angleDeg < STABILITY_MIN_ANGLE_DEG) {
            return Math.max(STABILITY_UNSTABLE_MIN,
                    STABILITY_UNSTABLE_BASE - deviation);
        }

        return Math.max(STABILITY_STABLE_MIN,
                STABILITY_STABLE_BASE - deviation);
    }

    /**
     * Calculates speed score based on time of flight. Faster (lower TOF) =
     * higher score.
     */
    private double calculateSpeedScore(ProjectileMotion.AngleEvaluation eval, double minTof, double tofRange) {
        if (tofRange < MIN_TOF_RANGE) {
            return SPEED_SCORE_DEFAULT;
        }

        double normalizedTof = (eval.timeOfFlight - minTof) / tofRange;
        return SPEED_SCORE_MAX - (normalizedTof * SPEED_SCORE_RANGE);
    }

    /**
     * Calculates clearance score based on maximum height. Higher trajectories
     * get better clearance scores for obstacle avoidance.
     */
    private double calculateClearanceScore(ProjectileMotion.AngleEvaluation eval, double maxHeight) {
        if (maxHeight < CLEARANCE_MIN_HEIGHT) {
            return CLEARANCE_SCORE_DEFAULT;
        }

        double relativeHeight = eval.maxHeight / maxHeight;
        return CLEARANCE_SCORE_BASE + (relativeHeight * CLEARANCE_SCORE_RANGE);
    }

    /**
     * Determines arc type based on pitch angle.
     */
    private ShotCandidate.ArcType determineArcType(double pitchDegrees) {
        if (pitchDegrees < ARC_LOW_MAX_DEG) {
            return ShotCandidate.ArcType.LOW_ARC;
        } else if (pitchDegrees > ARC_HIGH_MIN_DEG) {
            return ShotCandidate.ArcType.HIGH_ARC;
        } else if (pitchDegrees >= ARC_OPTIMAL_MIN_DEG
                && pitchDegrees <= ARC_OPTIMAL_MAX_DEG) {
            return ShotCandidate.ArcType.OPTIMAL_RANGE;
        } else {
            return ShotCandidate.ArcType.INTERMEDIATE;
        }
    }

    /**
     * Gets the best shot candidate according to the input's preference. This is
     * a convenience method that combines findAllCandidates with selection.
     *
     * @param input Shot input with preference setting
     * @return Best candidate according to preference, or empty if none found
     */
    public java.util.Optional<ShotCandidate> getBestCandidate(ShotInput input) {
        ShotCandidateList candidates = findAllCandidates(input);

        if (!candidates.hasValidSolution()) {
            return java.util.Optional.empty();
        }

        switch (input.getShotPreference()) {
            case FASTEST:
                return candidates.getFastest();
            case HIGH_CLEARANCE:
                return candidates.getMaxClearance();
            case MOST_STABLE:
                return candidates.getMostStable();
            case MIN_VELOCITY:
                return candidates.getMinVelocity();
            case MOST_ACCURATE:
                return candidates.getMostAccurate();
            case PREFER_HIGH_ARC:
                return candidates.getBestHighArc().or(candidates::getBest);
            case PREFER_LOW_ARC:
                return candidates.getBestLowArc().or(candidates::getBest);
            case AUTO:
            default:
                return candidates.getBestHit().or(candidates::getBest);
        }
    }

    /**
     * Solves for the best pitch angle given the input preferences. Returns just
     * the optimal pitch angle in radians.
     *
     * @param input Shot input parameters
     * @return Optimal pitch angle in radians, or NaN if no solution
     */
    public double solveBestPitch(ShotInput input) {
        return getBestCandidate(input)
                .map(ShotCandidate::getPitchAngleRadians)
                .orElse(Double.NaN);
    }

    /**
     * Solves for the best pitch angle in degrees.
     *
     * @param input Shot input parameters
     * @return Optimal pitch angle in degrees, or NaN if no solution
     */
    public double solveBestPitchDegrees(ShotInput input) {
        return getBestCandidate(input)
                .map(ShotCandidate::getPitchAngleDegrees)
                .orElse(Double.NaN);
    }

    // ==================== END NEW API ====================
    /**
     * Gets the optimal flywheel configuration for a velocity range.
     */
    public FlywheelConfig getOptimalFlywheel(double minVelocityMps, double maxVelocityMps) {
        FlywheelGenerator.GenerationResult result
                = flywheelGenerator.generateForVelocityRange(minVelocityMps, maxVelocityMps);

        return result.bestConfig != null ? result.bestConfig.config : null;
    }

    /**
     * Calculates confidence score for a solution.
     */
    private double calculateConfidence(double flywheelScore, boolean hitTarget,
            double marginOfError, double targetRadius,
            double crtScore) {
        double confidence = 0;

        confidence += Math.min(30, flywheelScore / 5);

        if (hitTarget) {
            /// God help me
            confidence += 40;
        } else {
            double relativeError = marginOfError / targetRadius;
            confidence += Math.max(0, 30 - relativeError * 20);
        }

        confidence += Math.min(20, crtScore / 5);

        confidence += 10;

        return Math.min(100, confidence);
    }

    /**
     * Clears the cached flywheel configuration.
     */
    public void clearCache() {
        cachedFlywheel = null;
    }

    /**
     * Sets a specific flywheel to use for all future solves.
     */
    public void setFlywheel(FlywheelConfig flywheel) {
        cachedFlywheel = flywheel;
    }

    public GamePiece getGamePiece() {
        return gamePiece;
    }

    public SolverConfig getConfig() {
        return config;
    }

    public ProjectileMotion getProjectileMotion() {
        return projectileMotion;
    }

    public FlywheelGenerator getFlywheelGenerator() {
        return flywheelGenerator;
    }

    public FlywheelConfig getCachedFlywheel() {
        return cachedFlywheel;
    }
}
