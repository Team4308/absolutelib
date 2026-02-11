package ca.team4308.absolutelib.math.trajectories.gamepiece;

import ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants;
import edu.wpi.first.math.util.Units;

/**
 * Represents a game piece with its physical properties. Base class for
 * different FRC game piece types.
 */
public class GamePiece {

    /**
     * Shape category for drag calculations.
     */
    public enum Shape {
        SPHERE,
        TORUS, // Ring-shaped (like 2024 notes)
        CUBE,
        CONE,
        CYLINDER
    }

    private final String name;
    private final String gameName;
    private final int gameYear;
    private final Shape shape;
    private final double diameterMeters;
    private final double massKg;
    private final double dragCoefficient;
    private final double coefficientOfRestitution;
    private final double compressionFactor; // How much the piece compresses under load (0-1)

    /**
     * Creates a game piece specification.
     *
     * @param name Name of the game piece
     * @param gameName FRC game name
     * @param gameYear FRC season year
     * @param shape Physical shape
     * @param diameterMeters Maximum diameter in meters
     * @param massKg Mass in kilograms
     * @param dragCoefficient Aerodynamic drag coefficient
     * @param coefficientOfRestitution Bounciness (0-1)
     * @param compressionFactor How much it compresses (0-1, where 0 = rigid)
     */
    public GamePiece(String name, String gameName, int gameYear, Shape shape,
            double diameterMeters, double massKg, double dragCoefficient,
            double coefficientOfRestitution, double compressionFactor) {
        this.name = name;
        this.gameName = gameName;
        this.gameYear = gameYear;
        this.shape = shape;
        this.diameterMeters = diameterMeters;
        this.massKg = massKg;
        this.dragCoefficient = dragCoefficient;
        this.coefficientOfRestitution = coefficientOfRestitution;
        this.compressionFactor = compressionFactor;
    }

    /**
     * Builder for creating game pieces with imperial units.
     */
    public static class Builder {

        private String name;
        private String gameName;
        private int gameYear;
        private Shape shape = Shape.SPHERE;
        private double diameterInches;
        private double massLbs;
        private double dragCoefficient = PhysicsConstants.SPHERE_DRAG_COEFFICIENT;
        private double cor = 0.6;
        private double compressionFactor = 0.1;

        public Builder name(String name) {
            this.name = name;
            return this;
        }

        public Builder game(String gameName, int year) {
            this.gameName = gameName;
            this.gameYear = year;
            return this;
        }

        public Builder shape(Shape shape) {
            this.shape = shape;
            return this;
        }

        public Builder diameterInches(double inches) {
            this.diameterInches = inches;
            return this;
        }

        public Builder massLbs(double lbs) {
            this.massLbs = lbs;
            return this;
        }

        public Builder massRange(double minLbs, double maxLbs) {
            this.massLbs = (minLbs + maxLbs) / 2.0;
            return this;
        }

        public Builder dragCoefficient(double cd) {
            this.dragCoefficient = cd;
            return this;
        }

        public Builder coefficientOfRestitution(double cor) {
            this.cor = cor;
            return this;
        }

        public Builder compressionFactor(double cf) {
            this.compressionFactor = cf;
            return this;
        }

        public GamePiece build() {
            return new GamePiece(
                    name, gameName, gameYear, shape,
                    Units.inchesToMeters(diameterInches),
                    Units.lbsToKilograms(massLbs),
                    dragCoefficient, cor, compressionFactor
            );
        }
    }

    /**
     * Creates a builder for fluent construction.
     */
    public static Builder builder() {
        return new Builder();
    }

    // Derived properties
    public double getRadiusMeters() {
        return diameterMeters / 2.0;
    }

    public double getDiameterInches() {
        return Units.metersToInches(diameterMeters);
    }

    /**
     * Returns the mass in pounds.
     */
    public double getMassLbs() {
        return massKg / Units.lbsToKilograms(1.0);
    }

    /**
     * Calculates cross-sectional area for drag calculations.
     */
    public double getCrossSectionalArea() {
        double radius = getRadiusMeters();
        switch (shape) {
            case SPHERE:
            case CUBE:
                return Math.PI * radius * radius;
            case TORUS:
                // Approximate as ellipse
                return Math.PI * radius * radius * 0.7;
            case CONE:
            case CYLINDER:
                return Math.PI * radius * radius;
            default:
                return Math.PI * radius * radius;
        }
    }

    /**
     * Calculates moment of inertia assuming uniform density.
     */
    public double getMomentOfInertia() {
        double radius = getRadiusMeters();
        switch (shape) {
            case SPHERE:
                // Hollow sphere approximation for foam ball
                return (2.0 / 3.0) * massKg * radius * radius;
            case CUBE:
                return (1.0 / 6.0) * massKg * (2 * radius) * (2 * radius);
            default:
                return (2.0 / 5.0) * massKg * radius * radius;
        }
    }

    /**
     * Calculates the compressed diameter under a given compression ratio.
     */
    public double getCompressedDiameter(double compressionRatio) {
        return diameterMeters * (1.0 - compressionRatio * compressionFactor);
    }

    /**
     * Estimates energy loss during compression/decompression. Based on
     * coefficient of restitution.
     */
    public double getEnergyTransferEfficiency() {
        // COR^2 gives energy retention
        return coefficientOfRestitution * coefficientOfRestitution;
    }

    // Getters
    public String getName() {
        return name;
    }

    public String getGameName() {
        return gameName;
    }

    public int getGameYear() {
        return gameYear;
    }

    public Shape getShape() {
        return shape;
    }

    public double getDiameterMeters() {
        return diameterMeters;
    }

    public double getMassKg() {
        return massKg;
    }

    public double getDragCoefficient() {
        return dragCoefficient;
    }

    public double getCoefficientOfRestitution() {
        return coefficientOfRestitution;
    }

    public double getCompressionFactor() {
        return compressionFactor;
    }

    @Override
    public String toString() {
        return String.format("%s (%d %s) - %.2f\" dia, %.3f lbs",
                name, gameYear, gameName, getDiameterInches(), getMassLbs());
    }
}
