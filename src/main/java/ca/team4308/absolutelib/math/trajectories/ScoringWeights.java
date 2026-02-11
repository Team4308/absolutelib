package ca.team4308.absolutelib.math.trajectories;

/**
 * Configurable weights for the trajectory solver's candidate scoring system.
 *
 * <p>Each weight is a multiplier applied to a scoring component. A weight of
 * {@code 0.0} disables that component entirely; {@code 1.0} is the default
 * contribution; values above {@code 1.0} increase its influence.</p>
 *
 * <h2>Components</h2>
 * <ul>
 *   <li><b>accuracyWeight</b> — how much the closest approach to the target
 *       matters. Higher = strongly prefer trajectories that pass close to the
 *       target center.</li>
 *   <li><b>hitBonusWeight</b> — flat bonus for trajectories where the ball
 *       enters the target radius (a direct hit).</li>
 *   <li><b>lowArcWeight</b> — preference for lower/flatter arcs. Higher =
 *       aggressively prefer lower pitch angles. Set to 0 if you want the solver
 *       to be arc-agnostic.</li>
 *   <li><b>speedWeight</b> — preference for shorter time of flight. Higher =
 *       prefer faster shots.</li>
 *   <li><b>stabilityWeight</b> — how much to reward angles near
 *       {@link #getOptimalAngleDegrees()}. Useful for mechanisms that are more
 *       accurate at a specific pitch.</li>
 *   <li><b>clearanceWeight</b> — bonus for clearing obstacles by a wider
 *       margin. Only active when obstacles are present.</li>
 *   <li><b>entryAngleWeight</b> — preference for steeper entry angles (ball
 *       coming down more vertically into the target). Higher = more aggressively
 *       prefer steep descents. Only has effect when trajectory crosses the
 *       rim plane.</li>
 * </ul>
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * ScoringWeights weights = ScoringWeights.builder()
 *     .lowArcWeight(2.0)        // strongly prefer flat shots
 *     .speedWeight(1.5)         // also prefer fast TOF
 *     .optimalAngleDegrees(25)  // mechanism is best around 25°
 *     .build();
 *
 * solver.setScoringWeights(weights);
 * }</pre>
 *
 * @see TrajectorySolver#setScoringWeights(ScoringWeights)
 * @deprecated Since 1.4.1 — the solver now uses a deterministic accuracy-first\n * scoring system. This class is retained for API compatibility only.\n */
@Deprecated
public final class ScoringWeights {

    private final double accuracyWeight;
    private final double hitBonusWeight;
    private final double lowArcWeight;
    private final double speedWeight;
    private final double stabilityWeight;
    private final double clearanceWeight;
    private final double entryAngleWeight;
    private final double optimalAngleDegrees;

    private ScoringWeights(Builder b) {
        this.accuracyWeight = b.accuracyWeight;
        this.hitBonusWeight = b.hitBonusWeight;
        this.lowArcWeight = b.lowArcWeight;
        this.speedWeight = b.speedWeight;
        this.stabilityWeight = b.stabilityWeight;
        this.clearanceWeight = b.clearanceWeight;
        this.entryAngleWeight = b.entryAngleWeight;
        this.optimalAngleDegrees = b.optimalAngleDegrees;
    }

    /** Weight for accuracy (closest approach to target). Default 1.0. */
    public double getAccuracyWeight() { return accuracyWeight; }

    /** Weight for the direct-hit bonus. Default 1.0. */
    public double getHitBonusWeight() { return hitBonusWeight; }

    /**
     * Weight for preferring lower arcs. Default 1.5.
     * Higher values more aggressively penalize steep angles.
     * Set to 0 to disable low-arc preference entirely.
     */
    public double getLowArcWeight() { return lowArcWeight; }

    /** Weight for preferring shorter time of flight. Default 1.0. */
    public double getSpeedWeight() { return speedWeight; }

    /** Weight for angles near {@link #getOptimalAngleDegrees()}. Default 0.5. */
    public double getStabilityWeight() { return stabilityWeight; }

    /** Weight for obstacle clearance margin. Default 0.5. */
    public double getClearanceWeight() { return clearanceWeight; }

    /**
     * Weight for preferring steeper entry angles. Default 1.0.
     * Higher values more aggressively reward trajectories that descend
     * steeply into the target. Set to 0 to disable entry-angle preference.
     */
    public double getEntryAngleWeight() { return entryAngleWeight; }

    /**
     * The pitch angle (degrees) considered mechanically optimal.
     * The stability component rewards angles near this value.
     * Default 30.0°.
     */
    public double getOptimalAngleDegrees() { return optimalAngleDegrees; }

    /**
     * Default weights: prefers low arcs, fast shots, and accuracy.
     */
    public static ScoringWeights defaults() {
        return builder().build();
    }

    /**
     * Weights optimized for flat/fast shots (e.g. close-range, no obstacles).
     */
    public static ScoringWeights flatShot() {
        return builder()
                .lowArcWeight(2.5)
                .speedWeight(2.0)
                .stabilityWeight(0.3)
                .optimalAngleDegrees(20)
                .build();
    }

    /**
     * Weights optimized for high-arc lob shots (e.g. over a defender or wall).
     */
    public static ScoringWeights highArc() {
        return builder()
                .lowArcWeight(0.0)
                .speedWeight(0.3)
                .clearanceWeight(2.0)
                .entryAngleWeight(2.0)
                .stabilityWeight(0.5)
                .optimalAngleDegrees(55)
                .build();
    }

    /** Creates a new builder with default weights. */
    public static Builder builder() {
        return new Builder();
    }

    /** Creates a builder pre-filled with the values from this instance. */
    public Builder toBuilder() {
        return new Builder()
                .accuracyWeight(accuracyWeight)
                .hitBonusWeight(hitBonusWeight)
                .lowArcWeight(lowArcWeight)
                .speedWeight(speedWeight)
                .stabilityWeight(stabilityWeight)
                .clearanceWeight(clearanceWeight)
                .entryAngleWeight(entryAngleWeight)
                .optimalAngleDegrees(optimalAngleDegrees);
    }

    @Override
    public String toString() {
        return String.format("ScoringWeights[accuracy=%.1f, hit=%.1f, lowArc=%.1f, speed=%.1f, stability=%.1f, clearance=%.1f, entryAngle=%.1f, optimal=%.0f\u00b0]",
                accuracyWeight, hitBonusWeight, lowArcWeight, speedWeight, stabilityWeight, clearanceWeight, entryAngleWeight, optimalAngleDegrees);
    }

    /**
     * Builder for {@link ScoringWeights}.
     */
    public static final class Builder {
        private double accuracyWeight = 1.0;
        private double hitBonusWeight = 1.0;
        private double lowArcWeight = 1.5;
        private double speedWeight = 1.0;
        private double stabilityWeight = 0.5;
        private double clearanceWeight = 0.5;
        private double entryAngleWeight = 1.0;
        private double optimalAngleDegrees = 30.0;

        /** Sets the accuracy (closest approach) weight. Default 1.0. */
        public Builder accuracyWeight(double val) { this.accuracyWeight = val; return this; }

        /** Sets the direct hit bonus weight. Default 1.0. */
        public Builder hitBonusWeight(double val) { this.hitBonusWeight = val; return this; }

        /** Sets the low-arc preference weight. Default 1.5. Higher = more aggressively prefer flat shots. */
        public Builder lowArcWeight(double val) { this.lowArcWeight = val; return this; }

        /** Sets the speed (short TOF) weight. Default 1.0. */
        public Builder speedWeight(double val) { this.speedWeight = val; return this; }

        /** Sets the angle stability weight. Default 0.5. */
        public Builder stabilityWeight(double val) { this.stabilityWeight = val; return this; }

        /** Sets the obstacle clearance weight. Default 0.5. */
        public Builder clearanceWeight(double val) { this.clearanceWeight = val; return this; }

        /** Sets the steep entry angle preference weight. Default 1.0. */
        public Builder entryAngleWeight(double val) { this.entryAngleWeight = val; return this; }

        /** Sets the optimal pitch angle in degrees. Default 30.0. */
        public Builder optimalAngleDegrees(double val) { this.optimalAngleDegrees = val; return this; }

        /** Builds the immutable {@link ScoringWeights}. */
        public ScoringWeights build() {
            return new ScoringWeights(this);
        }
    }
}
