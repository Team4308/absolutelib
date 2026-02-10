package ca.team4308.absolutelib.math.trajectories.shooter;

/**
 * A fixed-size, sorted lookup table for shot parameters indexed by distance.
 * 
 * <p>Each entry stores a tested distance, pitch angle, and RPM collected from
 * real robot testing. The table is kept small (8–15 entries) so it uses almost
 * no memory and is fast to search.</p>
 * 
 * <p>When the measured distance falls between two entries the table linearly
 * interpolates between them. When the distance is outside the range the
 * closest entry is returned (clamped).</p>
 * 
 * <h2>Usage</h2>
 * <pre>{@code
 * ShotLookupTable table = new ShotLookupTable()
 *     .addEntry(1.5, 75.0, 2000)
 *     .addEntry(2.5, 65.0, 2800)
 *     .addEntry(4.0, 55.0, 3800);
 * ShotParameters shot = table.lookup(3.0);
 * }</pre>
 */
public final class ShotLookupTable {

    /** Maximum number of entries the table can hold. */
    private static final int MAX_ENTRIES = 32;

    private final double[] distances = new double[MAX_ENTRIES];
    private final double[] pitchDegrees = new double[MAX_ENTRIES];
    private final double[] rpms = new double[MAX_ENTRIES];
    private int size = 0;

    /**
     * Adds a tested shot entry. Entries are insertion-sorted by distance.
     * 
     * @param distanceMeters horizontal distance to target in meters
     * @param pitchDeg       pitch angle in degrees
     * @param rpm            flywheel RPM
     * @return this table for chaining
     * @throws IllegalStateException if table is full ({@value MAX_ENTRIES} entries)
     */
    public ShotLookupTable addEntry(double distanceMeters, double pitchDeg, double rpm) {
        if (size >= MAX_ENTRIES) {
            throw new IllegalStateException("Lookup table is full (" + MAX_ENTRIES + " entries)");
        }
        int pos = size;
        for (int i = 0; i < size; i++) {
            if (distanceMeters < distances[i]) {
                pos = i;
                break;
            }
        }
        for (int i = size; i > pos; i--) {
            distances[i] = distances[i - 1];
            pitchDegrees[i] = pitchDegrees[i - 1];
            rpms[i] = rpms[i - 1];
        }
        distances[pos] = distanceMeters;
        pitchDegrees[pos] = pitchDeg;
        rpms[pos] = rpm;
        size++;
        return this;
    }

    /**
     * Looks up shot parameters for the given distance.
     * <ul>
     *   <li>If the distance is below the first entry, clamps to the first entry.</li>
     *   <li>If the distance is above the last entry, clamps to the last entry.</li>
     *   <li>Otherwise, linearly interpolates between the two surrounding entries.</li>
     * </ul>
     * 
     * @param distanceMeters measured distance to target
     * @return interpolated shot parameters tagged with {@link ShotParameters.Source#LOOKUP_TABLE}
     */
    public ShotParameters lookup(double distanceMeters) {
        if (size == 0) {
            return ShotParameters.invalid("Lookup table is empty");
        }
        if (distanceMeters <= distances[0]) {
            return new ShotParameters(pitchDegrees[0], rpms[0],
                    rpmToVelocity(rpms[0]), distances[0], ShotParameters.Source.LOOKUP_TABLE);
        }
        if (distanceMeters >= distances[size - 1]) {
            return new ShotParameters(pitchDegrees[size - 1], rpms[size - 1],
                    rpmToVelocity(rpms[size - 1]), distances[size - 1], ShotParameters.Source.LOOKUP_TABLE);
        }
        for (int i = 0; i < size - 1; i++) {
            if (distanceMeters >= distances[i] && distanceMeters <= distances[i + 1]) {
                double t = (distanceMeters - distances[i]) / (distances[i + 1] - distances[i]);
                double pitch = lerp(pitchDegrees[i], pitchDegrees[i + 1], t);
                double rpm = lerp(rpms[i], rpms[i + 1], t);
                return new ShotParameters(pitch, rpm, rpmToVelocity(rpm),
                        distanceMeters, ShotParameters.Source.LOOKUP_TABLE);
            }
        }
        return new ShotParameters(pitchDegrees[size - 1], rpms[size - 1],
                rpmToVelocity(rpms[size - 1]), distanceMeters, ShotParameters.Source.LOOKUP_TABLE);
    }

    /** Returns true if the table has at least one entry. */
    public boolean hasEntries() {
        return size > 0;
    }

    /** Returns the number of entries in the table. */
    public int getSize() {
        return size;
    }

    /** Returns the minimum distance covered by the table, or 0 if empty. */
    public double getMinDistance() {
        return size > 0 ? distances[0] : 0;
    }

    /** Returns the maximum distance covered by the table, or 0 if empty. */
    public double getMaxDistance() {
        return size > 0 ? distances[size - 1] : 0;
    }

    /**
     * Returns true if the given distance is within the table's range
     * (between the first and last entry, inclusive).
     */
    public boolean isInRange(double distanceMeters) {
        return size >= 2 && distanceMeters >= distances[0] && distanceMeters <= distances[size - 1];
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    /**
     * Rough RPM → exit velocity conversion.
     * Override with a proper model via {@link ShooterConfig#rpmToVelocityFactor}.
     */
    private static double rpmToVelocity(double rpm) {
        return rpm * 0.00532;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("ShotLookupTable[");
        for (int i = 0; i < size; i++) {
            if (i > 0) sb.append(", ");
            sb.append(String.format("%.1fm→%.1f°@%.0fRPM", distances[i], pitchDegrees[i], rpms[i]));
        }
        sb.append(']');
        return sb.toString();
    }
}
