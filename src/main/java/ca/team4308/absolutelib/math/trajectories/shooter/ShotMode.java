package ca.team4308.absolutelib.math.trajectories.shooter;

/**
 * The operating mode that controls how shot parameters are calculated.
 * 
 * <p>Each mode represents a different strategy with different tradeoffs
 * between reliability, accuracy, and computation cost.</p>
 */
public enum ShotMode {
    /**
     * Lookup table only.
     * Uses pre-tested values with interpolation. Fastest and most reliable.
     * No physics computation. Best for competition matches where fast,
     * predictable behavior is critical.
     */
    LOOKUP_ONLY,

    /**
     * Physics solver only.
     * Computes shot parameters from first principles using trajectory physics.
     * Most flexible but slowest and depends on model accuracy.
     * Falls back to a safe preset if the solver fails.
     */
    SOLVER_ONLY,

    /**
     * Lookup table primary, solver as backup.
     * If the distance is within the lookup table range, uses the table.
     * If outside the range or table is empty, falls back to the solver.
     * Good balance of reliability and flexibility.
     */
    LOOKUP_WITH_SOLVER_FALLBACK,

    /**
     * Solver primary, lookup table as backup.
     * Uses the solver for every shot. If the solver fails or returns an
     * invalid result, falls back to the lookup table.
     * Good for testing and tuning.
     */
    SOLVER_WITH_LOOKUP_FALLBACK,

    /**
     * Blended mode: averages lookup table and solver results when both
     * are available. Gives a result that is informed by both real-world
     * testing and physics modeling. The blend factor can be configured.
     */
    BLENDED,

    /**
     * Manual override: driver controls pitch and RPM directly.
     * Ignores all automatic calculations.
     */
    MANUAL
}
