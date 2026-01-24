package ca.team4308.absolutelib.math.trajectories.gamepiece;

import ca.team4308.absolutelib.math.trajectories.physics.PhysicsConstants;

/**
 * Predefined game pieces for various FRC games.
 * Includes the 2026 REBUILT game ball and historical pieces.
 */
public final class GamePieces {
    
    private GamePieces() {
        // Prevent instantiation
    }
    
    /**
     * 2026 REBUILT Game Ball.
     * High-density foam ball for shooting into targets.
     * 
     * Specifications:
     * - Shape: Sphere (Ball)
     * - Diameter: 5.91 inches
     * - Material: High-density foam
     * - Weight: 0.448-0.5 lbs (using average of 0.474 lbs)
     */
    public static final GamePiece REBUILT_2026_BALL = GamePiece.builder()
        .name("REBUILT Ball")
        .game("REBUILT", 2026)
        .shape(GamePiece.Shape.SPHERE)
        .diameterInches(5.91)
        .massRange(0.448, 0.5)
        .dragCoefficient(PhysicsConstants.FOAM_BALL_DRAG_COEFFICIENT)
        .coefficientOfRestitution(0.65) // Change??
        .compressionFactor(0.15)        // Change???
        .build();
    
    /**
     * 2024 CRESCENDO Note (Ring).
     */
    public static final GamePiece CRESCENDO_2024_NOTE = GamePiece.builder()
        .name("CRESCENDO Note")
        .game("CRESCENDO", 2024)
        .shape(GamePiece.Shape.TORUS)
        .diameterInches(14.0)  // Outer diameter
        .massLbs(0.5)
        .dragCoefficient(0.55)  // Torus has higher drag
        .coefficientOfRestitution(0.5)
        .compressionFactor(0.20)
        .build();
    
    /**
     * 2023 CHARGED UP Cone.
     */
    public static final GamePiece CHARGED_UP_2023_CONE = GamePiece.builder()
        .name("CHARGED UP Cone")
        .game("CHARGED UP", 2023)
        .shape(GamePiece.Shape.CONE)
        .diameterInches(8.375)  // Base diameter
        .massLbs(0.55)
        .dragCoefficient(0.45)
        .coefficientOfRestitution(0.3)
        .compressionFactor(0.10)
        .build();
    
    /**
     * 2023 CHARGED UP Cube.
     */
    public static final GamePiece CHARGED_UP_2023_CUBE = GamePiece.builder()
        .name("CHARGED UP Cube")
        .game("CHARGED UP", 2023)
        .shape(GamePiece.Shape.CUBE)
        .diameterInches(9.5)  // Side length
        .massLbs(0.6)
        .dragCoefficient(1.05) // Cubes have high drag
        .coefficientOfRestitution(0.4)
        .compressionFactor(0.12)
        .build();
    
    /**
     * 2022 RAPID REACT Cargo (Ball).
     */
    public static final GamePiece RAPID_REACT_2022_CARGO = GamePiece.builder()
        .name("RAPID REACT Cargo")
        .game("RAPID REACT", 2022)
        .shape(GamePiece.Shape.SPHERE)
        .diameterInches(9.5)
        .massLbs(0.57)
        .dragCoefficient(0.47)
        .coefficientOfRestitution(0.6)
        .compressionFactor(0.15)
        .build();
    
    /**
     * 2020/2021 INFINITE RECHARGE Power Cell.
     */
    public static final GamePiece INFINITE_RECHARGE_POWER_CELL = GamePiece.builder()
        .name("Power Cell")
        .game("INFINITE RECHARGE", 2020)
        .shape(GamePiece.Shape.SPHERE)
        .diameterInches(7.0)
        .massLbs(0.31)
        .dragCoefficient(0.47)
        .coefficientOfRestitution(0.7)
        .compressionFactor(0.12)
        .build();
    
    /**
     * 2019 DESTINATION: DEEP SPACE Cargo.
     */
    public static final GamePiece DEEP_SPACE_2019_CARGO = GamePiece.builder()
        .name("DEEP SPACE Cargo")
        .game("DESTINATION: DEEP SPACE", 2019)
        .shape(GamePiece.Shape.SPHERE)
        .diameterInches(13.0)
        .massLbs(0.6)
        .dragCoefficient(0.47)
        .coefficientOfRestitution(0.55)
        .compressionFactor(0.10)
        .build();
    
    /**
     * 2017 STEAMWORKS Fuel (Wiffle ball).
     */
    public static final GamePiece STEAMWORKS_2017_FUEL = GamePiece.builder()
        .name("STEAMWORKS Fuel")
        .game("STEAMWORKS", 2017)
        .shape(GamePiece.Shape.SPHERE)
        .diameterInches(5.0)
        .massLbs(0.12)
        .dragCoefficient(0.55) // Wiffle balls have higher drag
        .coefficientOfRestitution(0.4)
        .compressionFactor(0.05)
        .build();
    
    /**
     * All game pieces for iteration.
     */
    public static final GamePiece[] ALL_PIECES = {
        REBUILT_2026_BALL,
        CRESCENDO_2024_NOTE,
        CHARGED_UP_2023_CONE,
        CHARGED_UP_2023_CUBE,
        RAPID_REACT_2022_CARGO,
        INFINITE_RECHARGE_POWER_CELL,
        DEEP_SPACE_2019_CARGO,
        STEAMWORKS_2017_FUEL
    };
    
    /**
     * Spherical game pieces suitable for flywheel shooters.
     */
    public static final GamePiece[] SHOOTABLE_PIECES = {
        REBUILT_2026_BALL,
        RAPID_REACT_2022_CARGO,
        INFINITE_RECHARGE_POWER_CELL,
        DEEP_SPACE_2019_CARGO,
        STEAMWORKS_2017_FUEL
    };
    
    /**
     * Gets the default game piece for the current/upcoming season.
     */
    public static GamePiece getCurrent() {
        return REBUILT_2026_BALL;
    }
    
    /**
     * Gets a game piece by year.
     */
    public static GamePiece getByYear(int year) {
        for (GamePiece piece : ALL_PIECES) {
            if (piece.getGameYear() == year) {
                return piece;
            }
        }
        return null;
    }
    
    /**
     * Gets all shootable pieces (spheres that work with flywheels).
     */
    public static GamePiece[] getShootablePieces() {
        return SHOOTABLE_PIECES;
    }
}
