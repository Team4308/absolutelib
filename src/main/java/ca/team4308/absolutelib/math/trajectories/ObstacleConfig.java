package ca.team4308.absolutelib.math.trajectories;

/**
 * Configuration for field obstacles that trajectories must avoid.
 * Supports axis-aligned box obstacles (like hubs) with optional 
 * inner openings (like hoop openings on top).
 * 
 * <h2>Coordinate System</h2>
 * Uses field-relative coordinates in meters. Configure one obstacle per alliance
 * or set the position dynamically based on alliance color.
 * 
 * <h2>Hub Example (2026 REBUILT)</h2>
 * <pre>
 * ObstacleConfig hub = ObstacleConfig.builder()
 *     .position(4.03, 4.0)           // Hub center on field
 *     .baseSize(1.19, 1.19)          // 47in x 47in base
 *     .wallHeight(1.83)              // 72in hub edge
 *     .upperStructureHeight(0.41)    // 16.15in polycarbonate hexagon
 *     .openingDiameter(1.06)         // 41.7in hexagonal opening
 *     .enabled(true)
 *     .build();
 * </pre>
 * 
 * <h2>Alliance Mirroring</h2>
 * <pre>
 * ObstacleConfig blueHub = ObstacleConfig.builder()
 *     .position(4.03, 4.0).baseSize(1.19, 1.19)
 *     .wallHeight(1.83).upperStructureHeight(0.41)
 *     .openingDiameter(1.06).build();
 *     
 * // Mirror for red alliance (field is 16.54m long)    
 * ObstacleConfig redHub = blueHub.mirrorX(16.54);
 * </pre>
 */
public class ObstacleConfig {
    
    private final double centerX;
    private final double centerY;
    private final double baseSizeX;
    private final double baseSizeY;
    private final double wallHeight;
    private final double upperStructureHeight;
    private final double totalHeight;
    private final double openingDiameter;
    private final boolean enabled;
    private final double collisionMargin;
    
    private ObstacleConfig(Builder builder) {
        this.centerX = builder.centerX;
        this.centerY = builder.centerY;
        this.baseSizeX = builder.baseSizeX;
        this.baseSizeY = builder.baseSizeY;
        this.wallHeight = builder.wallHeight;
        this.upperStructureHeight = builder.upperStructureHeight;
        this.totalHeight = wallHeight + upperStructureHeight;
        this.openingDiameter = builder.openingDiameter;
        this.enabled = builder.enabled;
        this.collisionMargin = builder.collisionMargin;
    }
    
    /**
     * Checks if a 3D point collides with this obstacle.
     * 
     * @param x Point X position (meters)
     * @param y Point Y position (meters)
     * @param z Point Z position / height (meters)
     * @return true if the point is inside the obstacle
     */
    public boolean checkCollision(double x, double y, double z) {
        if (!enabled) return false;
        
        double halfX = baseSizeX / 2.0 + collisionMargin;
        double halfY = baseSizeY / 2.0 + collisionMargin;
        
        boolean inXRange = (x >= centerX - halfX) && (x <= centerX + halfX);
        boolean inYRange = (y >= centerY - halfY) && (y <= centerY + halfY);
        
        if (!inXRange || !inYRange) return false;

        if (z < wallHeight) {
            return true;
        }
        
        if (z < totalHeight && upperStructureHeight > 0) {
            double halfOpening = openingDiameter / 2.0;
            double distFromCenter = Math.sqrt(Math.pow(x - centerX, 2) + Math.pow(y - centerY, 2));

            if (distFromCenter > halfOpening) {
                return true;
            }
        }
        
        return false;
    }
    
    /**
     * Checks an entire trajectory for collisions with this obstacle.
     * Returns the index of the first collision point, or -1 if no collision.
     * 
     * @param xPoints X positions of trajectory points
     * @param yPoints Y positions of trajectory points
     * @param zPoints Z positions of trajectory points
     * @param numPoints Number of points to check
     * @return Index of first collision, or -1 if clear
     */
    public int checkTrajectory(double[] xPoints, double[] yPoints, double[] zPoints, int numPoints) {
        if (!enabled) return -1;
        
        for (int i = 0; i < numPoints; i++) {
            if (checkCollision(xPoints[i], yPoints[i], zPoints[i])) {
                return i;
            }
        }
        return -1;
    }
    
    /**
     * Returns the total height the trajectory must clear to avoid this obstacle.
     * Includes safety margin.
     */
    public double getClearanceHeight() {
        return totalHeight + collisionMargin;
    }
    
    /**
     * Checks if a horizontal position is within the obstacle's footprint.
     * Useful for determining if an "up and over" arc is needed.
     * 
     * @param x Point X position
     * @param y Point Y position
     * @return true if the point is above the obstacle's base footprint
     */
    public boolean isAboveFootprint(double x, double y) {
        if (!enabled) return false;
        double halfX = baseSizeX / 2.0 + collisionMargin;
        double halfY = baseSizeY / 2.0 + collisionMargin;
        return (x >= centerX - halfX) && (x <= centerX + halfX)
            && (y >= centerY - halfY) && (y <= centerY + halfY);
    }
    
    /**
     * Checks if a horizontal position is within the obstacle's top opening.
     * A ball descending through the opening should not be counted as a collision.
     * 
     * @param x Point X position
     * @param y Point Y position
     * @return true if the point is within the opening diameter
     */
    public boolean isWithinOpening(double x, double y) {
        if (!enabled || openingDiameter <= 0) return false;
        double halfOpening = openingDiameter / 2.0;
        double distFromCenter = Math.sqrt(Math.pow(x - centerX, 2) + Math.pow(y - centerY, 2));
        return distFromCenter <= halfOpening;
    }
    
    /**
     * Checks whether a straight-line path from shooter to target passes through
     * the obstacle's horizontal footprint. If so, the trajectory MUST arc over it.
     * 
     * @param shooterX Shooter X
     * @param shooterY Shooter Y
     * @param targetX Target X
     * @param targetY Target Y
     * @return true if the direct path crosses over the obstacle's footprint
     */
    public boolean pathCrossesObstacle(double shooterX, double shooterY, double targetX, double targetY) {
        if (!enabled) return false;
        
        double halfX = baseSizeX / 2.0 + collisionMargin;
        double halfY = baseSizeY / 2.0 + collisionMargin;
        
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;

        double tMinX, tMaxX, tMinY, tMaxY;
        
        if (Math.abs(dx) < 1e-9) {
            if (shooterX < centerX - halfX || shooterX > centerX + halfX) return false;
            tMinX = 0;
            tMaxX = 1;
        } else {
            double t1 = (centerX - halfX - shooterX) / dx;
            double t2 = (centerX + halfX - shooterX) / dx;
            tMinX = Math.min(t1, t2);
            tMaxX = Math.max(t1, t2);
        }
        
        if (Math.abs(dy) < 1e-9) {
            if (shooterY < centerY - halfY || shooterY > centerY + halfY) return false;
            tMinY = 0;
            tMaxY = 1;
        } else {
            double t1 = (centerY - halfY - shooterY) / dy;
            double t2 = (centerY + halfY - shooterY) / dy;
            tMinY = Math.min(t1, t2);
            tMaxY = Math.max(t1, t2);
        }
        
        double tMin = Math.max(tMinX, tMinY);
        double tMax = Math.min(tMaxX, tMaxY);
        
        return tMin <= tMax && tMax >= 0 && tMin <= 1;
    }
    
    /**
     * Creates a new ObstacleConfig mirrored across the X axis for the opposite alliance.
     * The field center X is at fieldLengthMeters / 2.
     * 
     * @param fieldLengthMeters Total field length in meters
     * @return New ObstacleConfig with mirrored X position
     */
    public ObstacleConfig mirrorX(double fieldLengthMeters) {
        return new Builder()
            .position(fieldLengthMeters - centerX, centerY)
            .baseSize(baseSizeX, baseSizeY)
            .wallHeight(wallHeight)
            .upperStructureHeight(upperStructureHeight)
            .openingDiameter(openingDiameter)
            .collisionMargin(collisionMargin)
            .enabled(enabled)
            .build();
    }
    
    /**
     * Creates a new ObstacleConfig mirrored across the Y axis.
     * 
     * @param fieldWidthMeters Total field width in meters
     * @return New ObstacleConfig with mirrored Y position
     */
    public ObstacleConfig mirrorY(double fieldWidthMeters) {
        return new Builder()
            .position(centerX, fieldWidthMeters - centerY)
            .baseSize(baseSizeX, baseSizeY)
            .wallHeight(wallHeight)
            .upperStructureHeight(upperStructureHeight)
            .openingDiameter(openingDiameter)
            .collisionMargin(collisionMargin)
            .enabled(enabled)
            .build();
    }
    
    public static Builder builder() {
        return new Builder();
    }
    
    public static class Builder {
        private double centerX = 0;
        private double centerY = 0;
        private double baseSizeX = 1.0;
        private double baseSizeY = 1.0;
        private double wallHeight = 1.0;
        private double upperStructureHeight = 0;
        private double openingDiameter = 0;
        private boolean enabled = true;
        private double collisionMargin = 0.05;

        /** Set obstacle center position in meters. */
        public Builder position(double x, double y) {
            this.centerX = x;
            this.centerY = y;
            return this;
        }
        
        /** Set obstacle base size in meters. */
        public Builder baseSize(double sizeX, double sizeY) {
            this.baseSizeX = sizeX;
            this.baseSizeY = sizeY;
            return this;
        }
        
        /** Set obstacle base size (square) in meters. */
        public Builder baseSize(double size) {
            this.baseSizeX = size;
            this.baseSizeY = size;
            return this;
        }
        
        /** Set wall height in meters (solid portion). */
        public Builder wallHeight(double height) {
            this.wallHeight = height;
            return this;
        }
        
        /** Set upper structure height above the wall (e.g. polycarbonate). */
        public Builder upperStructureHeight(double height) {
            this.upperStructureHeight = height;
            return this;
        }
        
        /** Set opening diameter in meters (e.g. hexagonal goal opening). */
        public Builder openingDiameter(double diameter) {
            this.openingDiameter = diameter;
            return this;
        }
        
        /** Set collision margin (extra buffer around obstacle). Default: 0.05m. */
        public Builder collisionMargin(double margin) {
            this.collisionMargin = margin;
            return this;
        }
        
        /** Enable or disable this obstacle. Default: true. */
        public Builder enabled(boolean enabled) {
            this.enabled = enabled;
            return this;
        }
        
        public ObstacleConfig build() {
            return new ObstacleConfig(this);
        }
    }
    
    public double getCenterX() { return centerX; }
    public double getCenterY() { return centerY; }
    public double getBaseSizeX() { return baseSizeX; }
    public double getBaseSizeY() { return baseSizeY; }
    public double getWallHeight() { return wallHeight; }
    public double getUpperStructureHeight() { return upperStructureHeight; }
    public double getTotalHeight() { return totalHeight; }
    public double getOpeningDiameter() { return openingDiameter; }
    public boolean isEnabled() { return enabled; }
    public double getCollisionMargin() { return collisionMargin; }
    
    @Override
    public String toString() {
        return String.format("Obstacle at (%.2f, %.2f) size=%.2fx%.2f wall=%.2fm upper=%.2fm total=%.2fm opening=%.2fm %s",
            centerX, centerY, baseSizeX, baseSizeY, wallHeight, upperStructureHeight, totalHeight, openingDiameter,
            enabled ? "ENABLED" : "DISABLED");
    }
}
