package ca.team4308.absolutelib.math.trajectories;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * A collection of shot candidates with utility methods for filtering and selecting.
 * Candidates are automatically sorted by confidence (highest first).
 */
public class ShotCandidateList {
    
    private final List<ShotCandidate> candidates;
    private final ShotInput input;
    private final boolean hasValidSolution;
    private final String statusMessage;
    
    /**
     * Creates a candidate list from computed candidates.
     */
    public ShotCandidateList(List<ShotCandidate> candidates, ShotInput input) {
        this.candidates = new ArrayList<>(candidates);
        Collections.sort(this.candidates); // Sort by confidence descending
        this.input = input;
        this.hasValidSolution = !candidates.isEmpty() && candidates.stream().anyMatch(ShotCandidate::hitsTarget);
        this.statusMessage = hasValidSolution ? "Valid solutions found" : 
            (candidates.isEmpty() ? "No trajectory solutions exist" : "No solutions hit target");
    }
    
    /**
     * Creates an empty/failed candidate list.
     */
    public static ShotCandidateList empty(ShotInput input, String reason) {
        ShotCandidateList list = new ShotCandidateList(Collections.emptyList(), input);
        return new ShotCandidateList(Collections.emptyList(), input) {
            @Override
            public String getStatusMessage() { return reason; }
        };
    }
    
    // ================= Selection Methods =================
    
    /**
     * Gets the best overall candidate (highest confidence).
     * @return Best candidate or empty if none exist
     */
    public Optional<ShotCandidate> getBest() {
        return candidates.isEmpty() ? Optional.empty() : Optional.of(candidates.get(0));
    }
    
    /**
     * Gets the best candidate that actually hits the target.
     * @return Best hitting candidate or empty if none hit
     */
    public Optional<ShotCandidate> getBestHit() {
        return candidates.stream()
            .filter(ShotCandidate::hitsTarget)
            .findFirst();
    }
    
    /**
     * Gets the best low-arc candidate (fastest time of flight).
     * @return Best low-arc candidate or empty if none exist
     */
    public Optional<ShotCandidate> getBestLowArc() {
        return candidates.stream()
            .filter(c -> c.getArcType() == ShotCandidate.ArcType.LOW_ARC || c.getPitchAngleDegrees() < 35)
            .filter(ShotCandidate::hitsTarget)
            .findFirst();
    }
    
    /**
     * Gets the best high-arc candidate (highest clearance).
     * @return Best high-arc candidate or empty if none exist
     */
    public Optional<ShotCandidate> getBestHighArc() {
        return candidates.stream()
            .filter(c -> c.getArcType() == ShotCandidate.ArcType.HIGH_ARC || c.getPitchAngleDegrees() > 50)
            .filter(ShotCandidate::hitsTarget)
            .findFirst();
    }
    
    /**
     * Gets the fastest shot (minimum time of flight).
     * @return Fastest candidate or empty if none exist
     */
    public Optional<ShotCandidate> getFastest() {
        return candidates.stream()
            .filter(ShotCandidate::hitsTarget)
            .min(Comparator.comparingDouble(ShotCandidate::getTimeOfFlightSeconds));
    }
    
    /**
     * Gets the most accurate shot (closest approach to target center).
     * @return Most accurate candidate or empty if none exist
     */
    public Optional<ShotCandidate> getMostAccurate() {
        return candidates.stream()
            .filter(ShotCandidate::hitsTarget)
            .max(Comparator.comparingDouble(ShotCandidate::getAccuracyScore));
    }
    
    /**
     * Gets the most stable shot (least sensitive to errors).
     * @return Most stable candidate or empty if none exist
     */
    public Optional<ShotCandidate> getMostStable() {
        return candidates.stream()
            .filter(ShotCandidate::hitsTarget)
            .max(Comparator.comparingDouble(ShotCandidate::getStabilityScore));
    }
    
    /**
     * Gets the shot with maximum clearance height.
     * @return Highest clearance candidate or empty if none exist
     */
    public Optional<ShotCandidate> getMaxClearance() {
        return candidates.stream()
            .filter(ShotCandidate::hitsTarget)
            .max(Comparator.comparingDouble(ShotCandidate::getMaxHeightMeters));
    }
    
    /**
     * Gets the shot requiring minimum velocity (easiest on mechanism).
     * @return Minimum velocity candidate or empty if none exist
     */
    public Optional<ShotCandidate> getMinVelocity() {
        return candidates.stream()
            .filter(ShotCandidate::hitsTarget)
            .min(Comparator.comparingDouble(ShotCandidate::getRequiredVelocityMps));
    }
    
    /**
     * Gets best candidate using custom weights.
     * @param accuracyWeight Weight for accuracy (0-1)
     * @param stabilityWeight Weight for stability (0-1)
     * @param speedWeight Weight for speed (0-1)
     * @param clearanceWeight Weight for clearance (0-1)
     * @return Best candidate with custom weights or empty if none exist
     */
    public Optional<ShotCandidate> getBestWithWeights(double accuracyWeight, double stabilityWeight,
                                                       double speedWeight, double clearanceWeight) {
        return candidates.stream()
            .filter(ShotCandidate::hitsTarget)
            .max(Comparator.comparingDouble(c -> 
                c.getConfidenceWithWeights(accuracyWeight, stabilityWeight, speedWeight, clearanceWeight)));
    }
    
    // ================= Filtering Methods =================
    
    /**
     * Gets all candidates that hit the target.
     * @return List of hitting candidates (sorted by confidence)
     */
    public List<ShotCandidate> getHittingCandidates() {
        return candidates.stream()
            .filter(ShotCandidate::hitsTarget)
            .collect(Collectors.toList());
    }
    
    /**
     * Gets candidates within a pitch angle range.
     * @param minDegrees Minimum pitch angle
     * @param maxDegrees Maximum pitch angle
     * @return Filtered candidates
     */
    public List<ShotCandidate> getCandidatesInPitchRange(double minDegrees, double maxDegrees) {
        return candidates.stream()
            .filter(c -> c.getPitchAngleDegrees() >= minDegrees && c.getPitchAngleDegrees() <= maxDegrees)
            .collect(Collectors.toList());
    }
    
    /**
     * Gets candidates with confidence above threshold.
     * @param minConfidence Minimum confidence (0-100)
     * @return High confidence candidates
     */
    public List<ShotCandidate> getCandidatesAboveConfidence(double minConfidence) {
        return candidates.stream()
            .filter(c -> c.getOverallConfidence() >= minConfidence)
            .collect(Collectors.toList());
    }
    
    /**
     * Gets candidates with velocity within range.
     * @param minVelocity Minimum velocity (m/s)
     * @param maxVelocity Maximum velocity (m/s)
     * @return Velocity-filtered candidates
     */
    public List<ShotCandidate> getCandidatesInVelocityRange(double minVelocity, double maxVelocity) {
        return candidates.stream()
            .filter(c -> c.getRequiredVelocityMps() >= minVelocity && c.getRequiredVelocityMps() <= maxVelocity)
            .collect(Collectors.toList());
    }
    
    /**
     * Gets candidates with time of flight under threshold.
     * @param maxSeconds Maximum time of flight
     * @return Fast candidates
     */
    public List<ShotCandidate> getCandidatesUnderTime(double maxSeconds) {
        return candidates.stream()
            .filter(c -> c.getTimeOfFlightSeconds() <= maxSeconds)
            .collect(Collectors.toList());
    }
    
    /**
     * Gets the top N candidates.
     * @param n Number of candidates to return
     * @return Top N candidates
     */
    public List<ShotCandidate> getTopN(int n) {
        return candidates.stream()
            .limit(n)
            .collect(Collectors.toList());
    }
    
    /**
     * Gets all low-arc candidates.
     * @return Low-arc candidates
     */
    public List<ShotCandidate> getLowArcCandidates() {
        return candidates.stream()
            .filter(ShotCandidate::isLowArc)
            .collect(Collectors.toList());
    }
    
    /**
     * Gets all high-arc candidates.
     * @return High-arc candidates
     */
    public List<ShotCandidate> getHighArcCandidates() {
        return candidates.stream()
            .filter(ShotCandidate::isHighArc)
            .collect(Collectors.toList());
    }
    
    // ================= Info Methods =================
    
    /**
     * Gets the total number of candidates.
     */
    public int size() {
        return candidates.size();
    }
    
    /**
     * Checks if any valid solutions exist.
     */
    public boolean hasValidSolution() {
        return hasValidSolution;
    }
    
    /**
     * Checks if the list is empty.
     */
    public boolean isEmpty() {
        return candidates.isEmpty();
    }
    
    /**
     * Gets the status message.
     */
    public String getStatusMessage() {
        return statusMessage;
    }
    
    /**
     * Gets the original input.
     */
    public ShotInput getInput() {
        return input;
    }
    
    /**
     * Gets all candidates (unmodifiable).
     */
    public List<ShotCandidate> getAllCandidates() {
        return Collections.unmodifiableList(candidates);
    }
    
    /**
     * Gets candidate by index.
     */
    public ShotCandidate get(int index) {
        return candidates.get(index);
    }
    
    /**
     * Gets the range of pitch angles covered.
     * @return [minPitch, maxPitch] in degrees
     */
    public double[] getPitchRange() {
        if (candidates.isEmpty()) return new double[]{0, 0};
        double min = candidates.stream().mapToDouble(ShotCandidate::getPitchAngleDegrees).min().orElse(0);
        double max = candidates.stream().mapToDouble(ShotCandidate::getPitchAngleDegrees).max().orElse(0);
        return new double[]{min, max};
    }
    
    /**
     * Gets the range of velocities covered.
     * @return [minVelocity, maxVelocity] in m/s
     */
    public double[] getVelocityRange() {
        if (candidates.isEmpty()) return new double[]{0, 0};
        double min = candidates.stream().mapToDouble(ShotCandidate::getRequiredVelocityMps).min().orElse(0);
        double max = candidates.stream().mapToDouble(ShotCandidate::getRequiredVelocityMps).max().orElse(0);
        return new double[]{min, max};
    }
    
    /**
     * Gets statistics about the candidates.
     */
    public String getStatistics() {
        if (candidates.isEmpty()) {
            return "No candidates generated";
        }
        
        long hits = candidates.stream().filter(ShotCandidate::hitsTarget).count();
        double[] pitch = getPitchRange();
        double[] velocity = getVelocityRange();
        double avgConf = candidates.stream().mapToDouble(ShotCandidate::getOverallConfidence).average().orElse(0);
        
        return String.format(
            "Candidates: %d (%d hits, %d misses)%n" +
            "Pitch range: %.1f deg - %.1f deg%n" +
            "Velocity range: %.1f - %.1f m/s%n" +
            "Average confidence: %.1f%%%n" +
            "Best confidence: %.1f%%",
            candidates.size(), hits, candidates.size() - hits,
            pitch[0], pitch[1],
            velocity[0], velocity[1],
            avgConf,
            candidates.isEmpty() ? 0 : candidates.get(0).getOverallConfidence()
        );
    }
    
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("ShotCandidateList[%d candidates, %s]%n", 
            candidates.size(), hasValidSolution ? "VALID" : "NO VALID SOLUTION"));
        
        int shown = Math.min(5, candidates.size());
        for (int i = 0; i < shown; i++) {
            sb.append(String.format("  %d. %s%n", i + 1, candidates.get(i)));
        }
        if (candidates.size() > 5) {
            sb.append(String.format("  ... and %d more%n", candidates.size() - 5));
        }
        
        return sb.toString();
    }
}
