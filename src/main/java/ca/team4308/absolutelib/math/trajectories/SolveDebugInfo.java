package ca.team4308.absolutelib.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion;

/**
 * Debug info from a solver run. Tracks accepted/rejected candidates
 * and their trajectory paths for visualization.
 *
 * Enable with {@link TrajectorySolver#setDebugEnabled(boolean)}.
 */

public class SolveDebugInfo {

    /**
     * Reason a candidate angle was rejected.
     */

    public enum RejectionReason {

        /** Candidate was accepted (not rejected). */

        NONE,

        /** Trajectory intersects an obstacle. */

        COLLISION,

        /** Apex height below the required minimum arc height. */

        ARC_TOO_LOW,

        /** Apex height below the required obstacle clearance. */

        CLEARANCE_TOO_LOW,

        /** Ball does not pass close enough to the target. */

        MISSED_TARGET,

        /** Ball passes above the target without descending into it. */

        FLYOVER
    }

    /**
     * Information about a single candidate angle tested by the solver.
     */

    public static class CandidateInfo {
        private final double pitchDegrees;
        private final RejectionReason rejection;
        private final double missDistance;
        private final double closestApproach;
        private final double maxHeight;
        private final double timeOfFlight;
        private final boolean hitTarget;
        private final ProjectileMotion.TrajectoryState[] trajectory;

        /**
         * Creates a candidate info entry recording the result of evaluating one pitch angle.
         *
         * @param pitchDegrees    the launch pitch angle tested
         * @param rejection       the rejection reason, or {@link RejectionReason#NONE} if accepted
         * @param missDistance    horizontal miss distance from target center (meters)
         * @param closestApproach closest approach distance to target (meters)
         * @param maxHeight       apex height of the trajectory (meters)
         * @param timeOfFlight    total time of flight (seconds)
         * @param hitTarget       true if the projectile hit the target
         * @param trajectory      sampled trajectory states for visualization
         */
        public CandidateInfo(double pitchDegrees, RejectionReason rejection, double missDistance,
                             double closestApproach, double maxHeight, double timeOfFlight,
                             boolean hitTarget, ProjectileMotion.TrajectoryState[] trajectory) {
            this.pitchDegrees = pitchDegrees;
            this.rejection = rejection;
            this.missDistance = missDistance;
            this.closestApproach = closestApproach;
            this.maxHeight = maxHeight;
            this.timeOfFlight = timeOfFlight;
            this.hitTarget = hitTarget;
            this.trajectory = trajectory;
        }

        public double getPitchDegrees() { return pitchDegrees; }
        public RejectionReason getRejection() { return rejection; }
        public boolean isAccepted() { return rejection == RejectionReason.NONE; }

        /** Horizontal distance from target center (meters). 0 = dead center. */

        public double getMissDistance() { return missDistance; }
        public double getClosestApproach() { return closestApproach; }
        public double getMaxHeight() { return maxHeight; }
        public double getTimeOfFlight() { return timeOfFlight; }
        public boolean getHitTarget() { return hitTarget; }
        public ProjectileMotion.TrajectoryState[] getTrajectory() { return trajectory; }

        @Override
        public String toString() {
            if (rejection == RejectionReason.NONE) {
                return String.format("  %5.1f°  ACCEPTED  miss=%.4fm  closest=%.3fm  TOF=%.3fs  maxH=%.2fm",
                        pitchDegrees, missDistance, closestApproach, timeOfFlight, maxHeight);
            } else {
                return String.format("  %5.1f°  %-18s  closest=%.3fm  TOF=%.3fs  maxH=%.2fm",
                        pitchDegrees, rejection, closestApproach, timeOfFlight, maxHeight);
            }
        }
    }

    private final List<CandidateInfo> candidates = new ArrayList<>();
    private int totalTested = 0;
    private int rejectedCollision = 0;
    private int rejectedArcTooLow = 0;
    private int rejectedClearance = 0;
    private int rejectedMiss = 0;
    private int rejectedFlyover = 0;
    private int accepted = 0;
    private double bestMissDistance = Double.POSITIVE_INFINITY;
    private double bestPitchDegrees = Double.NaN;

    /**
     * Records a candidate that was accepted (passed all filters).
     */

    public void recordAccepted(double pitchDeg, double missDistance, double closestApproach,
                               double maxHeight, double tof, boolean hitTarget,
                               ProjectileMotion.TrajectoryState[] trajectory) {
        totalTested++;
        accepted++;
        if (missDistance < bestMissDistance) {
            bestMissDistance = missDistance;
            bestPitchDegrees = pitchDeg;
        }
        candidates.add(new CandidateInfo(pitchDeg, RejectionReason.NONE, missDistance,
                closestApproach, maxHeight, tof, hitTarget, trajectory));
    }

    /**
     * Records a candidate that was rejected.
     */

    public void recordRejected(double pitchDeg, RejectionReason reason,
                               double closestApproach, double maxHeight, double tof,
                               boolean hitTarget, ProjectileMotion.TrajectoryState[] trajectory) {
        totalTested++;
        switch (reason) {
            case COLLISION:     rejectedCollision++; break;
            case ARC_TOO_LOW:   rejectedArcTooLow++; break;
            case CLEARANCE_TOO_LOW: rejectedClearance++; break;
            case MISSED_TARGET: rejectedMiss++; break;
            case FLYOVER:       rejectedFlyover++; break;
            default: break;
        }
        candidates.add(new CandidateInfo(pitchDeg, reason, 0, closestApproach,
                maxHeight, tof, hitTarget, trajectory));
    }

    public List<CandidateInfo> getCandidates() { return candidates; }

    /** Only candidates that passed all filters. */

    public List<CandidateInfo> getAcceptedCandidates() {
        List<CandidateInfo> result = new ArrayList<>();
        for (CandidateInfo c : candidates) {
            if (c.isAccepted()) result.add(c);
        }
        return result;
    }

    /** Only candidates that were rejected. */

    public List<CandidateInfo> getRejectedCandidates() {
        List<CandidateInfo> result = new ArrayList<>();
        for (CandidateInfo c : candidates) {
            if (!c.isAccepted()) result.add(c);
        }
        return result;
    }

    public int getTotalTested() { return totalTested; }
    public int getAcceptedCount() { return accepted; }
    public int getRejectedCollisionCount() { return rejectedCollision; }
    public int getRejectedArcTooLowCount() { return rejectedArcTooLow; }
    public int getRejectedClearanceCount() { return rejectedClearance; }
    public int getRejectedMissCount() { return rejectedMiss; }
    public int getRejectedFlyoverCount() { return rejectedFlyover; }
    public int getTotalRejected() { return totalTested - accepted; }

    /** Horizontal distance from target center for the best accepted candidate (meters). 0 = dead center. */

    public double getBestMissDistance() { return bestMissDistance; }
    public double getBestPitchDegrees() { return bestPitchDegrees; }

    /**
     * Returns a human-readable summary of the solve results.
     */

    public String getSummary() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Tested %d angles: %d accepted, %d rejected\n",
                totalTested, accepted, totalTested - accepted));
        sb.append(String.format("  Rejected by: collision=%d, arcTooLow=%d, clearance=%d, miss=%d, flyover=%d\n",
                rejectedCollision, rejectedArcTooLow, rejectedClearance, rejectedMiss, rejectedFlyover));
        if (!Double.isNaN(bestPitchDegrees)) {
            sb.append(String.format("  Best: %.1f° (miss=%.4fm)\n", bestPitchDegrees, bestMissDistance));
        } else {
            sb.append("  No valid solution found.\n");
        }
        return sb.toString();
    }

    /**
     * Returns a detailed table of all tested candidates.
     */

    public String getDetailedTable() {
        StringBuilder sb = new StringBuilder();
        sb.append("Pitch°   Status              Miss(m)  Closest    TOF      MaxHeight\n");
        sb.append("------   ------              -------  -------    ---      ---------\n");
        for (CandidateInfo c : candidates) {
            sb.append(c.toString()).append('\n');
        }
        return sb.toString();
    }

    @Override
    public String toString() {
        return getSummary();
    }
}
