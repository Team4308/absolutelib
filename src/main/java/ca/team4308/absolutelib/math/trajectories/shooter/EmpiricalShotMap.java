package ca.team4308.absolutelib.math.trajectories.shooter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ca.team4308.absolutelib.math.trajectories.impl.InterpolatingDoubleTreeMap;

/**
 * Empirical shot map built from real measured robot data.
 *
 * <p>Unlike the physics-based solver, which derives RPM from flywheel energy
 * transfer models, this class stores actual <b>measured</b> (distance → pitch, RPM)
 * tuples and interpolates between them. This eliminates the systematic RPM
 * overestimation caused by idealized compression, slip, and exit-velocity models.
 *
 * <p><b>Why this exists:</b> On a real FRC shooter, the relationship between
 * flywheel RPM and ball exit velocity is affected by:
 * <ul>
 *   <li>Ball compression hysteresis (energy absorbed by the foam/rubber)</li>
 *   <li>Wheel slip that varies with ball age, temperature, and wear</li>
 *   <li>Exit geometry losses at the hood/barrel opening</li>
 *   <li>Motor back-EMF under load reducing effective surface speed</li>
 *   <li>Non-ideal ball spin transfer</li>
 * </ul>
 * These effects are extremely difficult to model accurately, so the best approach
 * is to <b>measure real shots and interpolate</b>.
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * EmpiricalShotMap map = new EmpiricalShotMap()
 *     .addPoint(1.264, 7.5, 2100)
 *     .addPoint(1.300, 7.5, 2150)
 *     .addPoint(1.500, 10.0, 2250)
 *     .addPoint(1.710, 22.24, 2100)
 *     .addPoint(2.000, 17.4, 2100)
 *     .addPoint(3.000, 12.5, 2100);
 *
 * EmpiricalShotMap.QueryResult result = map.query(2.5);
 * double rpm = result.rpm;       // Interpolated RPM
 * double pitch = result.pitchDegrees; // Interpolated pitch
 * }</pre>
 *
 * <h2>Anomaly Detection</h2>
 * <p>The map detects non-monotonic RPM or pitch segments and flags them as
 * anomalous. This helps catch measurement errors or unusual shooter behavior.
 * Anomalous segments still return interpolated values but with a warning flag.
 *
 * <h2>Tuning Guide</h2>
 * <ol>
 *   <li>Start with 5-6 points spread across your shooting range</li>
 *   <li>Add more points where behavior changes rapidly (e.g., angle transitions)</li>
 *   <li>Re-measure when changing ball batches, wheel wear, or hood geometry</li>
 *   <li>Use {@link #getAnomalies()} to identify suspicious segments</li>
 * </ol>
 *
 * @see ShooterSystem
 * @see ShotLookupTable
 */
public final class EmpiricalShotMap {

    private final InterpolatingDoubleTreeMap pitchMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    private final List<DataPoint> rawPoints = new ArrayList<>();
    private final List<String> anomalies = new ArrayList<>();

    private double minRpmClamp = 1800.0;
    private double maxRpmClamp = 6000.0;

    private double minDistance = Double.MAX_VALUE;
    private double maxDistance = -Double.MAX_VALUE;

    /**
     * A single measured data point.
     */
    public static final class DataPoint implements Comparable<DataPoint> {
        public final double distanceMeters;
        public final double pitchDegrees;
        public final double rpm;

        public DataPoint(double distanceMeters, double pitchDegrees, double rpm) {
            this.distanceMeters = distanceMeters;
            this.pitchDegrees = pitchDegrees;
            this.rpm = rpm;
        }

        @Override
        public int compareTo(DataPoint other) {
            return Double.compare(this.distanceMeters, other.distanceMeters);
        }

        @Override
        public String toString() {
            return String.format("%.3fm → %.1f° / %.0f RPM", distanceMeters, pitchDegrees, rpm);
        }
    }

    /**
     * Result of a map query, including diagnostic flags.
     */
    public static final class QueryResult {
        /** Interpolated pitch angle in degrees. */
        public final double pitchDegrees;
        /** Interpolated flywheel RPM. */
        public final double rpm;
        /** Raw RPM before clamping. */
        public final double rawRpm;
        /** True if the query distance was within the map range. */
        public final boolean inRange;
        /** True if the RPM was clamped to the configured bounds. */
        public final boolean wasClamped;
        /** True if the query falls in an anomalous segment. */
        public final boolean inAnomalousSegment;
        /** Distance queried (meters). */
        public final double distanceMeters;

        QueryResult(double pitchDegrees, double rpm, double rawRpm,
                    boolean inRange, boolean wasClamped, boolean inAnomalousSegment,
                    double distanceMeters) {
            this.pitchDegrees = pitchDegrees;
            this.rpm = rpm;
            this.rawRpm = rawRpm;
            this.inRange = inRange;
            this.wasClamped = wasClamped;
            this.inAnomalousSegment = inAnomalousSegment;
            this.distanceMeters = distanceMeters;
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            sb.append(String.format("%.3fm → %.1f° / %.0f RPM", distanceMeters, pitchDegrees, rpm));
            if (wasClamped) sb.append(" [CLAMPED]");
            if (!inRange) sb.append(" [EXTRAPOLATED]");
            if (inAnomalousSegment) sb.append(" [ANOMALY]");
            return sb.toString();
        }
    }

    /**
     * Creates an empty empirical shot map with default RPM clamping (1800–3000).
     */
    public EmpiricalShotMap() {}

    /**
     * Creates an empirical shot map with custom RPM clamping bounds.
     *
     * @param minRpmClamp minimum allowed RPM output
     * @param maxRpmClamp maximum allowed RPM output
     */
    public EmpiricalShotMap(double minRpmClamp, double maxRpmClamp) {
        this.minRpmClamp = minRpmClamp;
        this.maxRpmClamp = maxRpmClamp;
    }

    /**
     * Adds a measured data point.
     *
     * @param distanceMeters horizontal distance where the shot was taken
     * @param pitchDegrees   pitch angle that made the shot
     * @param rpm            flywheel RPM that made the shot
     * @return this map for chaining
     */
    public EmpiricalShotMap addPoint(double distanceMeters, double pitchDegrees, double rpm) {
        pitchMap.put(distanceMeters, pitchDegrees);
        rpmMap.put(distanceMeters, rpm);
        rawPoints.add(new DataPoint(distanceMeters, pitchDegrees, rpm));

        minDistance = Math.min(minDistance, distanceMeters);
        maxDistance = Math.max(maxDistance, distanceMeters);

        // Rebuild anomaly list whenever a new point is added
        detectAnomalies();
        return this;
    }

    /**
     * Queries the map for interpolated shot parameters at the given distance.
     *
     * @param distanceMeters horizontal distance to target
     * @return query result with interpolated values and diagnostic flags
     */
    public QueryResult query(double distanceMeters) {
        if (rawPoints.isEmpty()) {
            return new QueryResult(0, 0, 0, false, false, false, distanceMeters);
        }

        boolean inRange = distanceMeters >= minDistance && distanceMeters <= maxDistance;

        // InterpolatingDoubleTreeMap extrapolates outside its range, which is acceptable
        // for small extrapolations but should be flagged.
        Double pitchObj = pitchMap.get(distanceMeters);
        Double rpmObj = rpmMap.get(distanceMeters);

        double pitch = pitchObj != null ? pitchObj : 0;
        double rawRpm = rpmObj != null ? rpmObj : 0;

        // Clamp RPM to configured bounds
        double clampedRpm = Math.max(minRpmClamp, Math.min(maxRpmClamp, rawRpm));
        boolean wasClamped = Math.abs(clampedRpm - rawRpm) > 1.0;

        boolean inAnomaly = isInAnomalousSegment(distanceMeters);

        return new QueryResult(pitch, clampedRpm, rawRpm, inRange, wasClamped, inAnomaly, distanceMeters);
    }

    /**
     * Returns true if the map has at least one data point.
     */
    public boolean hasData() {
        return !rawPoints.isEmpty();
    }

    /**
     * Returns the number of data points in the map.
     */
    public int size() {
        return rawPoints.size();
    }

    /**
     * Returns the minimum distance in the map, or 0 if empty.
     */
    public double getMinDistance() {
        return rawPoints.isEmpty() ? 0 : minDistance;
    }

    /**
     * Returns the maximum distance in the map, or 0 if empty.
     */
    public double getMaxDistance() {
        return rawPoints.isEmpty() ? 0 : maxDistance;
    }

    /**
     * Returns true if the given distance falls within the map's data range.
     */
    public boolean isInRange(double distanceMeters) {
        return rawPoints.size() >= 2
                && distanceMeters >= minDistance
                && distanceMeters <= maxDistance;
    }

    /**
     * Returns an unmodifiable list of detected anomaly descriptions.
     * Each string describes a segment where RPM or pitch behaves unexpectedly.
     */
    public List<String> getAnomalies() {
        return Collections.unmodifiableList(anomalies);
    }

    /**
     * Returns true if there are any detected anomalies.
     */
    public boolean hasAnomalies() {
        return !anomalies.isEmpty();
    }

    /**
     * Returns an unmodifiable copy of all raw data points, sorted by distance.
     */
    public List<DataPoint> getDataPoints() {
        List<DataPoint> sorted = new ArrayList<>(rawPoints);
        Collections.sort(sorted);
        return Collections.unmodifiableList(sorted);
    }

    /**
     * Sets the RPM clamping bounds.
     *
     * @param minRpm minimum allowed RPM
     * @param maxRpm maximum allowed RPM
     * @return this map for chaining
     */
    public EmpiricalShotMap setRpmClamp(double minRpm, double maxRpm) {
        this.minRpmClamp = minRpm;
        this.maxRpmClamp = maxRpm;
        return this;
    }

    /**
     * Returns the minimum RPM clamp value.
     */
    public double getMinRpmClamp() {
        return minRpmClamp;
    }

    /**
     * Returns the maximum RPM clamp value.
     */
    public double getMaxRpmClamp() {
        return maxRpmClamp;
    }

    // ---- Anomaly detection ----

    /**
     * Checks if a query distance falls in a segment flagged as anomalous.
     * An anomalous segment is one where RPM decreases as distance increases,
     * or where pitch changes direction unexpectedly.
     */
    private boolean isInAnomalousSegment(double distanceMeters) {
        List<DataPoint> sorted = new ArrayList<>(rawPoints);
        Collections.sort(sorted);

        for (int i = 0; i < sorted.size() - 1; i++) {
            DataPoint a = sorted.get(i);
            DataPoint b = sorted.get(i + 1);

            if (distanceMeters >= a.distanceMeters && distanceMeters <= b.distanceMeters) {
                // RPM decreasing with increasing distance is unusual but not always wrong
                // (e.g., close-range shots may need more speed for steep entry angles).
                // Flag it only if the drop is significant (>200 RPM over a short distance).
                double rpmDelta = b.rpm - a.rpm;
                double distDelta = b.distanceMeters - a.distanceMeters;
                if (distDelta > 0.01 && rpmDelta < -200.0) {
                    return true;
                }

                // Pitch reversal: normally pitch changes monotonically or stays flat.
                // A large reversal in a short distance segment is suspicious.
                double pitchDelta = b.pitchDegrees - a.pitchDegrees;
                if (distDelta > 0.01 && Math.abs(pitchDelta) > 15.0
                        && i > 0 && i < sorted.size() - 2) {
                    // Check if the direction reversed compared to neighboring segments
                    double prevPitchDelta = a.pitchDegrees - sorted.get(i - 1).pitchDegrees;
                    if (prevPitchDelta * pitchDelta < 0) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    /**
     * Rebuilds the anomaly description list from the current data points.
     */
    private void detectAnomalies() {
        anomalies.clear();

        if (rawPoints.size() < 2) {
            return;
        }

        List<DataPoint> sorted = new ArrayList<>(rawPoints);
        Collections.sort(sorted);

        for (int i = 0; i < sorted.size() - 1; i++) {
            DataPoint a = sorted.get(i);
            DataPoint b = sorted.get(i + 1);
            double distDelta = b.distanceMeters - a.distanceMeters;

            if (distDelta < 0.001) {
                anomalies.add(String.format(
                        "Duplicate distance: %.3fm has multiple entries", a.distanceMeters));
                continue;
            }

            double rpmDelta = b.rpm - a.rpm;
            if (rpmDelta < -200.0) {
                anomalies.add(String.format(
                        "RPM drops %.0f (%.0f→%.0f) between %.3fm and %.3fm — "
                        + "verify these are from the same session",
                        -rpmDelta, a.rpm, b.rpm, a.distanceMeters, b.distanceMeters));
            }

            double pitchDelta = b.pitchDegrees - a.pitchDegrees;
            if (i > 0) {
                double prevPitchDelta = a.pitchDegrees - sorted.get(i - 1).pitchDegrees;
                if (prevPitchDelta * pitchDelta < 0 && Math.abs(pitchDelta) > 10.0) {
                    anomalies.add(String.format(
                            "Pitch reversal at %.3fm (%.1f°→%.1f°→%.1f°) — "
                            + "may indicate measurement error or intentional arc change",
                            a.distanceMeters,
                            sorted.get(i - 1).pitchDegrees, a.pitchDegrees, b.pitchDegrees));
                }
            }
        }
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("EmpiricalShotMap[%d points, %.2f-%.2fm, clamp %.0f-%.0f RPM]",
                rawPoints.size(), minDistance, maxDistance, minRpmClamp, maxRpmClamp));
        if (!anomalies.isEmpty()) {
            sb.append(String.format(" (%d anomalies)", anomalies.size()));
        }
        return sb.toString();
    }
}
