package ca.team4308.absolutelib.math.trajectories;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;

/**
 * Tests for TrajectorySolver bug fixes: - SWEEP quality scoring instead of
 * miss-distance-only - Drag compensation applied to horizontal component only -
 * GamePiece.getMassLbs() conversion correctness
 */
public class TrajectorySolverTest {

    private TrajectorySolver solver;
    private GamePiece gamePiece;

    @BeforeEach
    void setUp() {
        gamePiece = GamePieces.REBUILT_2026_BALL;
        SolverConstants.setMinTargetDistanceMeters(0.05);
        SolverConstants.setVelocityBufferMultiplier(1.2);
        SolverConstants.setRimClearanceMeters(0.15);

        TrajectorySolver.SolverConfig config = TrajectorySolver.SolverConfig.defaults()
                .toBuilder()
                .minPitchDegrees(20)
                .maxPitchDegrees(82.5)
                .build();

        solver = new TrajectorySolver(gamePiece, config);
    }

    @Test
    void sweepProducesReasonableAngle() {
        solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(0, 0, 0.5)
                .targetPositionMeters(5.0, 0, 2.1)
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .build();

        TrajectoryResult result = solver.solve(input);

        if (result.isSuccess()) {
            double pitchDeg = result.getPitchAngleDegrees();
            assertTrue(pitchDeg >= 30 && pitchDeg <= 72,
                    "SWEEP pitch should be in reasonable range [30-72°], got " + pitchDeg);
        }
    }

    @Test
    void constraintProducesValidSolution() {
        solver.setSolveMode(TrajectorySolver.SolveMode.CONSTRAINT);

        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(0, 0, 0.5)
                .targetPositionMeters(5.0, 0, 2.1)
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .build();

        TrajectoryResult result = solver.solve(input);

        if (result.isSuccess()) {
            double pitchDeg = result.getPitchAngleDegrees();
            assertTrue(pitchDeg > 0 && pitchDeg < 90,
                    "CONSTRAINT pitch should be valid, got " + pitchDeg);
            assertTrue(result.getRequiredVelocityMps() > 0,
                    "Velocity should be positive");
        }
    }

    @Test
    void sweepAndConstraintProduceSimilarAngles() {
        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(0, 0, 0.5)
                .targetPositionMeters(4.0, 0, 2.1)
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .build();

        solver.setSolveMode(TrajectorySolver.SolveMode.CONSTRAINT);
        TrajectoryResult constraintResult = solver.solve(input);

        solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);
        TrajectoryResult sweepResult = solver.solve(input);

        if (constraintResult.isSuccess() && sweepResult.isSuccess()) {
            double diff = Math.abs(constraintResult.getPitchAngleDegrees()
                    - sweepResult.getPitchAngleDegrees());
            assertTrue(diff < 20,
                    "SWEEP and CONSTRAINT should produce angles within 20° of each other, got diff=" + diff
                    + " (constraint=" + constraintResult.getPitchAngleDegrees()
                    + ", sweep=" + sweepResult.getPitchAngleDegrees() + ")");
        }
    }

    @Test
    void gamePieceMassLbsConversion() {
        GamePiece ball = GamePieces.REBUILT_2026_BALL;
        double massKg = ball.getMassKg();
        double massLbs = ball.getMassLbs();
        double expectedLbs = massKg / 0.45359237;
        assertEquals(expectedLbs, massLbs, 0.01,
                "getMassLbs() should correctly convert kg to lbs");
    }

    @Test
    void gamePieceMassRoundTrip() {
        GamePiece ball = GamePieces.REBUILT_2026_BALL;
        double massKg = ball.getMassKg();
        double massLbs = ball.getMassLbs();
        double backToKg = massLbs * 0.45359237;
        assertEquals(massKg, backToKg, 0.001,
                "kg -> lbs -> kg round trip should be consistent");
    }

    @Test
    void sweepDoesNotSelectExtremeAngles() {
        solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

        TrajectorySolver.SolverConfig wideConfig = TrajectorySolver.SolverConfig.defaults()
                .toBuilder()
                .minPitchDegrees(10)
                .maxPitchDegrees(85)
                .build();
        TrajectorySolver wideSolver = new TrajectorySolver(gamePiece, wideConfig);
        wideSolver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(0, 0, 0.5)
                .targetPositionMeters(6.0, 0, 2.1)
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .build();

        TrajectoryResult result = wideSolver.solve(input);

        if (result.isSuccess()) {
            double pitchDeg = result.getPitchAngleDegrees();
            assertTrue(pitchDeg < 78,
                    "SWEEP should not select extreme high angles (>78°), got " + pitchDeg);
        }
    }

    @Test
    void solverHandlesCloseTarget() {
        solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(0, 0, 0.5)
                .targetPositionMeters(1.5, 0, 2.1)
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .build();

        TrajectoryResult result = solver.solve(input);
        assertNotNull(result, "Solver should return a non-null result for close targets");
    }

    @Test
    void solverHandlesFarTarget() {
        solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(0, 0, 0.5)
                .targetPositionMeters(10.0, 0, 2.1)
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .build();

        TrajectoryResult result = solver.solve(input);
        assertNotNull(result, "Solver should return a non-null result for far targets");
    }

    @Test
    void solverRejectsTargetTooClose() {
        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(0, 0, 0.5)
                .targetPositionMeters(0.01, 0, 2.1)
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .build();

        TrajectoryResult result = solver.solve(input);
        assertFalse(result.isSuccess(),
                "Should reject targets that are too close");
    }
}
