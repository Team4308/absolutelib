package ca.team4308.absolutelib.math.trajectories;

/**
 * Represents a possible shot solution with confidence scoring.
 * Multiple candidates can be generated for a single target, allowing
 * the caller to choose the best option based on various criteria.
 */
public class ShotCandidate implements Comparable<ShotCandidate> {
    
    /**
     * Type of trajectory arc.
     */
    public enum ArcType {
        /** Low, flat trajectory - faster time of flight, lower clearance */
        LOW_ARC,
        /** High, lobbing trajectory - slower, higher clearance */
        HIGH_ARC,
        /** Optimal 45-degree-ish trajectory for maximum range */
        OPTIMAL_RANGE,
        /** Custom angle that still hits target */
        INTERMEDIATE
    }
    
    // Shot parameters
    private final double pitchAngleRadians;
    private final double requiredVelocityMps;
    private final double yawAngleRadians;
    
    // Trajectory metrics
    private final double timeOfFlightSeconds;
    private final double maxHeightMeters;
    private final double closestApproachMeters;
    private final boolean hitsTarget;
    
    // Classification
    private final ArcType arcType;
    
    // Confidence scoring (0-100)
    private final double overallConfidence;
    private final double accuracyScore;      // How close to center of target
    private final double stabilityScore;     // How sensitive to small errors
    private final double speedScore;         // Faster = better for moving targets
    private final double clearanceScore;     // Obstacle clearance potential
    
    /**
     * Full constructor with all parameters.
     */
    public ShotCandidate(
            double pitchAngleRadians, double requiredVelocityMps, double yawAngleRadians,
            double timeOfFlightSeconds, double maxHeightMeters, double closestApproachMeters,
            boolean hitsTarget, ArcType arcType,
            double accuracyScore, double stabilityScore, double speedScore, double clearanceScore) {
        this.pitchAngleRadians = pitchAngleRadians;
        this.requiredVelocityMps = requiredVelocityMps;
        this.yawAngleRadians = yawAngleRadians;
        this.timeOfFlightSeconds = timeOfFlightSeconds;
        this.maxHeightMeters = maxHeightMeters;
        this.closestApproachMeters = closestApproachMeters;
        this.hitsTarget = hitsTarget;
        this.arcType = arcType;
        this.accuracyScore = accuracyScore;
        this.stabilityScore = stabilityScore;
        this.speedScore = speedScore;
        this.clearanceScore = clearanceScore;
        
        // Calculate overall confidence as weighted average
        this.overallConfidence = calculateOverallConfidence();
    }
    
    /**
     * Builder for easier construction.
     */
    public static class Builder {
        private double pitchAngleRadians;
        private double requiredVelocityMps;
        private double yawAngleRadians;
        private double timeOfFlightSeconds;
        private double maxHeightMeters;
        private double closestApproachMeters;
        private boolean hitsTarget;
        private ArcType arcType = ArcType.HIGH_ARC;
        private double accuracyScore = 50;
        private double stabilityScore = 50;
        private double speedScore = 50;
        private double clearanceScore = 50;
        
        public Builder pitchAngleRadians(double val) { this.pitchAngleRadians = val; return this; }
        public Builder pitchAngleDegrees(double val) { this.pitchAngleRadians = Math.toRadians(val); return this; }
        public Builder requiredVelocityMps(double val) { this.requiredVelocityMps = val; return this; }
        public Builder yawAngleRadians(double val) { this.yawAngleRadians = val; return this; }
        public Builder timeOfFlightSeconds(double val) { this.timeOfFlightSeconds = val; return this; }
        public Builder maxHeightMeters(double val) { this.maxHeightMeters = val; return this; }
        public Builder closestApproachMeters(double val) { this.closestApproachMeters = val; return this; }
        public Builder hitsTarget(boolean val) { this.hitsTarget = val; return this; }
        public Builder arcType(ArcType val) { this.arcType = val; return this; }
        public Builder accuracyScore(double val) { this.accuracyScore = val; return this; }
        public Builder stabilityScore(double val) { this.stabilityScore = val; return this; }
        public Builder speedScore(double val) { this.speedScore = val; return this; }
        public Builder clearanceScore(double val) { this.clearanceScore = val; return this; }
        
        public ShotCandidate build() {
            return new ShotCandidate(
                pitchAngleRadians, requiredVelocityMps, yawAngleRadians,
                timeOfFlightSeconds, maxHeightMeters, closestApproachMeters,
                hitsTarget, arcType,
                accuracyScore, stabilityScore, speedScore, clearanceScore
            );
        }
    }
    
    public static Builder builder() {
        return new Builder();
    }
    
    /**
     * Calculates overall confidence from component scores.
     * Weights can be adjusted based on game requirements.
     */
    private double calculateOverallConfidence() {
        // Default weights - accuracy is most important
        double accuracyWeight = 0.40;
        double stabilityWeight = 0.25;
        double speedWeight = 0.20;
        double clearanceWeight = 0.15;
        
        double weighted = accuracyScore * accuracyWeight +
                         stabilityScore * stabilityWeight +
                         speedScore * speedWeight +
                         clearanceScore * clearanceWeight;
        
        // Penalty for not actually hitting target
        if (!hitsTarget) {
            weighted *= 0.5;
        }
        
        return Math.max(0, Math.min(100, weighted));
    }
    
    /**
     * Recalculates confidence with custom weights.
     * @param accuracyWeight Weight for accuracy (0-1)
     * @param stabilityWeight Weight for stability (0-1)
     * @param speedWeight Weight for speed (0-1)
     * @param clearanceWeight Weight for clearance (0-1)
     * @return Recalculated confidence score
     */
    public double getConfidenceWithWeights(double accuracyWeight, double stabilityWeight,
                                           double speedWeight, double clearanceWeight) {
        double total = accuracyWeight + stabilityWeight + speedWeight + clearanceWeight;
        if (total < 0.001) return 0;
        
        double weighted = (accuracyScore * accuracyWeight +
                          stabilityScore * stabilityWeight +
                          speedScore * speedWeight +
                          clearanceScore * clearanceWeight) / total;
        
        if (!hitsTarget) {
            weighted *= 0.5;
        }
        
        return Math.max(0, Math.min(100, weighted));
    }
    
    // Convenience getters
    
    /**
     * Gets pitch angle in degrees.
     */
    public double getPitchAngleDegrees() {
        return Math.toDegrees(pitchAngleRadians);
    }
    
    /**
     * Gets yaw angle in degrees.
     */
    public double getYawAngleDegrees() {
        return Math.toDegrees(yawAngleRadians);
    }
    
    /**
     * Gets required velocity in feet per second.
     */
    public double getRequiredVelocityFps() {
        return requiredVelocityMps * 3.28084;
    }
    
    /**
     * Gets max height in feet.
     */
    public double getMaxHeightFeet() {
        return maxHeightMeters * 3.28084;
    }
    
    /**
     * Checks if this is a low arc shot (pitch &lt; 30 degrees).
     */
    public boolean isLowArc() {
        return arcType == ArcType.LOW_ARC || getPitchAngleDegrees() < 30;
    }
    
    /**
     * Checks if this is a high arc shot (pitch > 50 degrees).
     */
    public boolean isHighArc() {
        return arcType == ArcType.HIGH_ARC || getPitchAngleDegrees() > 50;
    }
    
    // Standard getters
    public double getPitchAngleRadians() { return pitchAngleRadians; }
    public double getRequiredVelocityMps() { return requiredVelocityMps; }
    public double getYawAngleRadians() { return yawAngleRadians; }
    public double getTimeOfFlightSeconds() { return timeOfFlightSeconds; }
    public double getMaxHeightMeters() { return maxHeightMeters; }
    public double getClosestApproachMeters() { return closestApproachMeters; }
    public boolean hitsTarget() { return hitsTarget; }
    public ArcType getArcType() { return arcType; }
    public double getOverallConfidence() { return overallConfidence; }
    public double getAccuracyScore() { return accuracyScore; }
    public double getStabilityScore() { return stabilityScore; }
    public double getSpeedScore() { return speedScore; }
    public double getClearanceScore() { return clearanceScore; }
    
    /**
     * Compares by overall confidence (higher is better).
     */
    @Override
    public int compareTo(ShotCandidate other) {
        return Double.compare(other.overallConfidence, this.overallConfidence);
    }
    
    @Override
    public String toString() {
        return String.format("ShotCandidate[%.1f° %s, %.1fm/s, TOF=%.2fs, conf=%.0f%% (%s)]",
            getPitchAngleDegrees(), arcType, requiredVelocityMps, 
            timeOfFlightSeconds, overallConfidence, hitsTarget ? "HIT" : "MISS");
    }
    
    /**
     * Returns a detailed multi-line description.
     */
    public String toDetailedString() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("=== Shot Candidate (%s) ===%n", arcType));
        sb.append(String.format("  Pitch: %.2f° (%.4f rad)%n", getPitchAngleDegrees(), pitchAngleRadians));
        sb.append(String.format("  Yaw: %.2f°%n", getYawAngleDegrees()));
        sb.append(String.format("  Velocity: %.2f m/s (%.1f fps)%n", requiredVelocityMps, getRequiredVelocityFps()));
        sb.append(String.format("  Time of Flight: %.3f s%n", timeOfFlightSeconds));
        sb.append(String.format("  Max Height: %.2f m (%.1f ft)%n", maxHeightMeters, getMaxHeightFeet()));
        sb.append(String.format("  Closest Approach: %.3f m%n", closestApproachMeters));
        sb.append(String.format("  Hits Target: %s%n", hitsTarget ? "YES" : "NO"));
        sb.append(String.format("  Confidence Breakdown:%n"));
        sb.append(String.format("    Overall: %.1f%%%n", overallConfidence));
        sb.append(String.format("    Accuracy: %.1f%%%n", accuracyScore));
        sb.append(String.format("    Stability: %.1f%%%n", stabilityScore));
        sb.append(String.format("    Speed: %.1f%%%n", speedScore));
        sb.append(String.format("    Clearance: %.1f%%%n", clearanceScore));
        return sb.toString();
    }
}
