package ca.team4308.absolutelib.math.trajectories.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Distance-indexed lookup table for pre-tuned shot parameters using WPILib's
 * {@link InterpolatingDoubleTreeMap} for smooth interpolation between entries.
 *
 * <p>Each entry maps a horizontal distance (meters) to a pitch angle (degrees),
 * flywheel RPM, and optionally a time-of-flight estimate. Between entries,
 * values are linearly interpolated, matching the approach used by
 * Mechanical Advantage's LaunchCalculator.
 *
 * <p>Typical usage:
 * <pre>{@code
 * ShotLookupTable table = new ShotLookupTable()
 *     .addEntry(2.0, 55.0, 3000)
 *     .addEntry(3.5, 45.0, 3800)
 *     .addEntry(5.0, 35.0, 4500, 0.62);
 *
 * ShotParameters params = table.lookup(2.8);
 * }</pre>
 *
 * @see ShooterSystem
 * @see ShotParameters
 */
public final class ShotLookupTable {

    private final InterpolatingDoubleTreeMap pitchMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
    private int size = 0;
    private boolean hasTofData = false;
    private double minDistance = Double.MAX_VALUE;
    private double maxDistance = Double.MIN_VALUE;
    private double rpmToVelocityFactor = 0.00532;

    /**
     * Creates a lookup table with the default RPM-to-velocity conversion factor
     * ({@code 0.00532}, matching a ~4" compliant wheel at 1:1 gear ratio).
     */
    public ShotLookupTable() {}

    /**
     * Creates a lookup table with a custom RPM-to-velocity conversion factor.
     * This should match the factor configured in {@link ShooterConfig} so that
     * {@code exitVelocityMps} values from the table are consistent with the
     * rest of the shooter pipeline.
     *
     * @param rpmToVelocityFactor multiplier converting RPM to m/s
     */
    public ShotLookupTable(double rpmToVelocityFactor) {
        this.rpmToVelocityFactor = rpmToVelocityFactor;
    }

    /**
     * Adds a shot entry for a given distance.
     *
     * @param distanceMeters horizontal distance to target in meters
     * @param pitchDeg       pitch angle in degrees
     * @param rpm            flywheel RPM
     * @return this table for chaining
     */
    public ShotLookupTable addEntry(double distanceMeters, double pitchDeg, double rpm) {
        pitchMap.put(distanceMeters, pitchDeg);
        rpmMap.put(distanceMeters, rpm);
        size++;
        minDistance = Math.min(minDistance, distanceMeters);
        maxDistance = Math.max(maxDistance, distanceMeters);
        return this;
    }

    /**
     * Adds a shot entry with time-of-flight estimate.
     *
     * @param distanceMeters horizontal distance to target in meters
     * @param pitchDeg       pitch angle in degrees
     * @param rpm            flywheel RPM
     * @param tofSeconds     estimated time of flight in seconds
     * @return this table for chaining
     */
    public ShotLookupTable addEntry(double distanceMeters, double pitchDeg, double rpm, double tofSeconds) {
        addEntry(distanceMeters, pitchDeg, rpm);
        tofMap.put(distanceMeters, tofSeconds);
        hasTofData = true;
        return this;
    }

    /**
     * Looks up interpolated shot parameters for the given distance.
     *
     * @param distanceMeters horizontal distance to target
     * @return interpolated shot parameters, or invalid if table is empty
     */
    public ShotParameters lookup(double distanceMeters) {
        if (size == 0) {
            return ShotParameters.invalid("Lookup table is empty");
        }
        double clamped = Math.max(minDistance, Math.min(maxDistance, distanceMeters));
        double pitch = pitchMap.get(clamped);
        double rpm = rpmMap.get(clamped);
        return new ShotParameters(pitch, rpm, rpmToVelocity(rpm),
                distanceMeters, ShotParameters.Source.LOOKUP_TABLE);
    }

    /**
     * Returns the interpolated time-of-flight estimate for the given distance.
     *
     * @param distanceMeters horizontal distance to target
     * @return estimated TOF in seconds, or 0 if no TOF data exists
     */
    public double getTimeOfFlight(double distanceMeters) {
        if (size == 0 || !hasTofData) {
            return 0;
        }
        double clamped = Math.max(minDistance, Math.min(maxDistance, distanceMeters));
        return tofMap.get(clamped);
    }

    /** Returns {@code true} if at least one entry has been added. */
    public boolean hasEntries() {
        return size > 0;
    }

    /** Returns the number of entries in the table. */
    public int getSize() {
        return size;
    }

    /** Returns the minimum distance entry in the table, or 0 if empty. */
    public double getMinDistance() {
        return size > 0 ? minDistance : 0;
    }

    /** Returns the maximum distance entry in the table, or 0 if empty. */
    public double getMaxDistance() {
        return size > 0 ? maxDistance : 0;
    }

    /**
     * Returns {@code true} if the given distance falls within the table's
     * min/max range and there are at least 2 entries for interpolation.
     */
    public boolean isInRange(double distanceMeters) {
        return size >= 2 && distanceMeters >= minDistance && distanceMeters <= maxDistance;
    }

    /** Returns the underlying pitch interpolation map. */
    public InterpolatingDoubleTreeMap getPitchMap() {
        return pitchMap;
    }

    /** Returns the underlying RPM interpolation map. */
    public InterpolatingDoubleTreeMap getRpmMap() {
        return rpmMap;
    }

    /** Returns the underlying time-of-flight interpolation map. */
    public InterpolatingDoubleTreeMap getTofMap() {
        return tofMap;
    }

    private double rpmToVelocity(double rpm) {
        return rpm * rpmToVelocityFactor;
    }

    @Override
    public String toString() {
        return String.format("ShotLookupTable[%d entries, %.1f-%.1fm]", size, minDistance, maxDistance);
    }
}
