package ca.team4308.absolutelib.math.trajectories.network;

import ca.team4308.absolutelib.math.trajectories.shooter.ShotLookupTable;
import java.util.HashMap;
import java.util.Map;

public class DefaultShotTable {

    /**
     * Returns the default shot lookup table for Team 4308 2026.
     * Uses WPILib's InterpolatingDoubleTreeMap for smooth interpolation.
     * Uses auto-calibrated RPM-to-velocity conversion factor from actual shot physics.
     * Use this when no shot table is received from the robot.
     */
    public static ShotLookupTable getDefault() {
        double rpmToVelocityFactor = calibrateRpmFactor();
        ShotLookupTable table = new ShotLookupTable(rpmToVelocityFactor);
        
        // Distance (m) -> Pitch (degrees), RPM
        table.addEntry(1.3, 90 - 8.5, 1700.0);
        table.addEntry(1.6, 90 - 12.5, 1750.0);
        table.addEntry(1.9, 90 - 13.5, 1780.0);
        table.addEntry(2.3, 90 - 14.5, 1830.0);
        table.addEntry(2.6, 90 - 15.5, 1890.0);
        table.addEntry(2.9, 90 - 16.5, 1980.0);
        table.addEntry(3.3, 90 - 17.0, 2080.0);
        table.addEntry(3.6, 90 - 17.5, 2160.0);
        table.addEntry(3.9, 90 - 18.0, 2180.0);
        table.addEntry(4.3, 90 - 18.5, 2240.0);
        table.addEntry(4.6, 90 - 19.0, 2305.0);
        table.addEntry(4.9, 90 - 19.0, 2380.0);
        table.addEntry(5.2, 90 - 19.5, 2420.0);
        table.addEntry(5.5, 90 - 20.0, 2480.0);
        table.addEntry(6.0, 90 - 20.0, 2550.0);
        table.addEntry(7.0, 90 - 21.0, 2650.0);
        table.addEntry(8.0, 90 - 22.0, 2750.0);
        table.addEntry(9.0, 90 - 22.5, 2850.0);
        table.addEntry(10.0, 90 - 23.0, 2950.0);
        table.addEntry(11.0, 90 - 23.5, 3050.0);
        table.addEntry(12.0, 90 - 24.0, 3150.0);
        table.addEntry(13.0, 90 - 24.5, 3250.0);
        table.addEntry(14.0, 90 - 25.0, 3350.0);
        table.addEntry(15.0, 90 - 25.5, 3450.0);
        table.addEntry(16.0, 90 - 26.0, 3550.0);
        
        return table;
    }

    /**
     * Calibrates the RPM-to-velocity conversion factor from the actual shot data.
     * Uses projectile motion equations to reverse-calculate exit velocity from known good shots.
     *
     * @return calibrated RPM-to-velocity conversion factor (m/s per RPM)
     */
    private static double calibrateRpmFactor() {
        double shooterHeight = 0.5;  // meters
        double targetHeight = 2.64;  // meters (2026 speaker height)
        
        // Reference calibration points: {distance_m, pitch_deg, rpm}
        double[][] points = {
            {2.6, 90 - 15.5, 1890.0},
            {4.3, 90 - 18.5, 2240.0},
            {6.0, 90 - 20.0, 2550.0},
            {10.0, 90 - 23.0, 2950.0},
        };
        
        double sumFactor = 0;
        int validCount = 0;
        
        for (int i = 0; i < points.length; i++) {
            double distance = points[i][0];
            double pitchDeg = points[i][1];
            double rpm = points[i][2];
            
            try {
                double velocity = estimateRequiredVelocity(distance, pitchDeg, shooterHeight, targetHeight);
                if (rpm > 0) {
                    double factor = velocity / rpm;
                    sumFactor += factor;
                    validCount++;
                }
            } catch (Exception e) {
                // Skip invalid calibration points
            }
        }
        
        if (validCount > 0) {
            double calibratedFactor = sumFactor / validCount;
            System.out.println("Auto-calibrated RPM factor: " + calibratedFactor);
            return calibratedFactor;
        }
        
        System.out.println("RPM calibration failed, using default factor: 0.01532");
        return 0.01532;
    }

    /**
     * Estimates the required exit velocity to reach a target at given distance and angle.
     * Uses basic projectile motion: x = v*cos(θ)*t, y = v*sin(θ)*t - 0.5*g*t²
     * Solves for v given x, θ, and y.
     *
     * @param horizontalDistance horizontal distance to target (meters)
     * @param pitchDegrees launch angle (degrees)
     * @param shooterHeight shooter position height (meters)
     * @param targetHeight target height (meters)
     * @return required exit velocity (m/s)
     */
    private static double estimateRequiredVelocity(
            double horizontalDistance, double pitchDegrees,
            double shooterHeight, double targetHeight) {
        
        double pitchRad = Math.toRadians(pitchDegrees);
        double g = 9.81; // gravity (m/s²)
        double deltaY = targetHeight - shooterHeight;
        
        double cosPitch = Math.cos(pitchRad);
        double tanPitch = Math.tan(pitchRad);
        
        // Quadratic formula: a*t² + b*t + c = 0
        // where y = deltaY = v*sinPitch*t - 0.5*g*t²
        // and x = horizontalDistance = v*cosPitch*t
        // Therefore: t = x / (v*cosPitch)
        // Substitute: deltaY = x*tanPitch - 0.5*g*x²/(v*cosPitch)²
        // Rearrange: v² = (g*x²) / (2*cosPitch²*(x*tanPitch - deltaY))
        
        double denominator = 2.0 * cosPitch * cosPitch * (horizontalDistance * tanPitch - deltaY);
        
        if (denominator <= 0) {
            throw new IllegalArgumentException("Unreachable target: angle too shallow or target too high");
        }
        
        double velocitySquared = (g * horizontalDistance * horizontalDistance) / denominator;
        
        if (velocitySquared < 0) {
            throw new IllegalArgumentException("Invalid trajectory parameters");
        }
        
        return Math.sqrt(velocitySquared);
    }

    /**
     * Returns the default shot table as a HashMap suitable for network transmission.
     * Formats each entry as: "distance_meters" -> [pitch_degrees, rpm]
     */
    public static Map<String, Object> getDefaultAsNetworkPayload() {
        Map<String, Object> payload = new HashMap<>();
        ShotLookupTable table = getDefault();
        
        // Extract entries from the pitch and rpm maps
        for (Double distance : table.getPitchMap().keySet()) {
            Double pitch = table.getPitchMap().get(distance);
            Double rpm = table.getRpmMap().get(distance);
            if (pitch != null && rpm != null) {
                String distanceKey = String.format("%.1f", distance);
                payload.put(distanceKey, new double[]{pitch, rpm});
            }
        }
        
        return payload;
    }

    /**
     * Looks up interpolated shot parameters for a given distance.
     * Returns null if distance is outside the valid range.
     */
    public static double[] lookup(double distanceMeters) {
        ShotLookupTable table = getDefault();
        
        if (!table.isInRange(distanceMeters)) {
            return null;
        }
        
        Double pitch = table.getPitchMap().get(distanceMeters);
        Double rpm = table.getRpmMap().get(distanceMeters);
        
        if (pitch == null || rpm == null) {
            return null;
        }
        
        return new double[]{pitch, rpm};
    }
}
