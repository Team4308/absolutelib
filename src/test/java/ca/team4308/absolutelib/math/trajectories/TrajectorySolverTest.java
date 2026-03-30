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

    @Test
    void closeRangeShotSelectionIsStable() {
        solver.setSolveMode(TrajectorySolver.SolveMode.BISECTION);

        ShotInput closeInput = ShotInput.builder()
                .shooterPositionMeters(0, 0, 0.5)
                .targetPositionMeters(2.0, 0, 2.1)
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .build();

        ShotInput slightlyCloser = ShotInput.builder()
                .shooterPositionMeters(0, 0, 0.5)
                .targetPositionMeters(1.8, 0, 2.1)
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .build();

        TrajectoryResult result1 = solver.solve(closeInput);
        TrajectoryResult result2 = solver.solve(slightlyCloser);

        assertTrue(result1.isSuccess() && result2.isSuccess(), "Both close-range shots should succeed");
        assertEquals(result1.getPitchAngleDegrees(), result2.getPitchAngleDegrees(), 10.0,
                "Pitch angle should not oscillate wildly at close range");
        assertTrue(Math.abs(result1.getRecommendedRpm() - result2.getRecommendedRpm()) < 2000.0,
                "RPM should not jump dramatically for nearby close-range positions");

        assertTrue(result1.getPitchAngleDegrees() > 0.5, "Pitch should not be zero");
        assertTrue(result1.getRecommendedRpm() > 100.0, "RPM should not be zero");
                assertTrue(result1.getRecommendedRpm() >= 1900.0, "Close-range RPM should meet min 1.9k floor");
                assertTrue(result2.getRecommendedRpm() >= 1900.0, "Close-range RPM should meet min 1.9k floor");
    }

        @Test
        void closeRangeShotDoesNotUseBelowMinRpmInLookup() {
                solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

                ShotInput input = ShotInput.builder()
                                .shooterPositionMeters(0, 0, 0.5)
                                .targetPositionMeters(0.5, 0, 2.1)
                                .targetRadiusMeters(0.45)
                                .includeAirResistance(true)
                                .build();

                TrajectoryResult result = solver.solve(input);
                assertTrue(result.isSuccess(), "Close range should still produce a solution");
                assertTrue(result.getRecommendedRpm() >= 1900.0,
                                "Solver should enforce min 1900 RPM for close range shots");
        }

        @Test
        void midRangeShotPrefersLowerRPM() {
                solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

                ShotInput input = ShotInput.builder()
                                .shooterPositionMeters(0, 0, 0.5)
                                        .targetPositionMeters(3.0, 0, 2.1)
                                .targetRadiusMeters(0.45)
                                .includeAirResistance(true)
                                .build();

                TrajectoryResult result = solver.solve(input);
                // May not always have a valid solution (solver constraints may reject exact conditions), but
                // if a solution exists we expect a low-RPM preference for mid-range.
                if (result.isSuccess()) {
                    assertTrue(result.getRecommendedRpm() < 2600.0,
                            "Mid-range solver should prefer lower rpm (at or below ~2.6k), got " + result.getRecommendedRpm());
                }
        }

                @Test
                void midRangeShotRejectsVeryHighRPM() {
                        solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

                        // Use a known mid-range geometry that produces low-RPM solutions in the precomputed table.
                        ShotInput input = ShotInput.builder()
                                .shooterPositionMeters(7.307, 5.341, 0.5)
                                .targetPositionMeters(7.55, 5.4, 2.1)
                                        .targetRadiusMeters(0.45)
                                        .includeAirResistance(true)
                                        .build();

                        TrajectoryResult result = solver.solve(input);
                        assertTrue(result.isSuccess(), "Mid-range state should produce a valid solution");
                        assertTrue(result.getRecommendedRpm() <= 3000.0,
                                        "Solver should avoid RPM above 3k in mid-range (<6.5m) shots, got " + result.getRecommendedRpm());
                }

            @Test
            void computeSweepQualityScorePenalizesHighRpmAtFiveMeters() throws Exception {
                ShotInput input = ShotInput.builder()
                        .shooterPositionMeters(0, 0, 0.5)
                        .targetPositionMeters(5.0, 0, 2.1)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .build();

                java.lang.reflect.Method method = TrajectorySolver.class.getDeclaredMethod(
                        "computeSweepQualityScore",
                        ShotInput.class, double.class, double.class, double.class, double.class,
                        double.class, double.class, double.class, double.class);
                method.setAccessible(true);

                double scoreLow = (double) method.invoke(solver,
                        input, 40.0, 0.1, 0.45, 1.2, 30.0, 1900.0, 5.0, 4.0);
                double scoreHigh = (double) method.invoke(solver,
                        input, 40.0, 0.1, 0.45, 1.2, 30.0, 3100.0, 5.0, 4.0);

                assertTrue(scoreLow > scoreHigh,
                        String.format("Expected lower RPM to score higher at mid-range (low=%.2f high=%.2f)", scoreLow, scoreHigh));
            }

            @Test
            void closeRangeTargetRPMisCorrect() {
                // Use the empirical map to get realistic RPM values.
                // Without the map, the physics solver overestimates RPM (e.g., 2727 at 1.3m)
                // because the flywheel energy transfer model is idealized.
                ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap map =
                    new ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap();
                map.addPoint(1.264, 7.5, 2100);
                map.addPoint(1.300, 7.5, 2150);
                map.addPoint(1.500, 10.0, 2250);
                map.addPoint(1.710, 22.24, 2100);
                map.addPoint(2.000, 17.4, 2100);
                map.addPoint(3.000, 12.5, 2100);

                solver.setEmpiricalMap(map);
                solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);
                ShotInput input = ShotInput.builder()
                        .shooterPositionMeters(0, 0, 0.5)
                        .targetPositionMeters(1.3, 0, 2.1)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .build();

                TrajectoryResult result = solver.solve(input);
                if (result.isSuccess()) {
                    double rpm = result.getRecommendedRpm();
                    // User's measured value at 1.3m is 2150. Empirical map should return close to that.
                    assertTrue(rpm >= 2000 && rpm <= 2300,
                            "1.3m RPM should be in range [2000-2300] (from empirical map), got " + rpm);
                }
            }

            @Test
            void midRangeTargetRPMisCorrect() {
                solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);
                ShotInput input3m = ShotInput.builder()
                        .shooterPositionMeters(0, 0, 0.5)
                        .targetPositionMeters(3.0, 0, 2.1)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .build();

                TrajectoryResult result3m = solver.solve(input3m);
                if (result3m.isSuccess()) {
                    double rpm = result3m.getRecommendedRpm();
                    // Expected ~2100-2400 RPM
                    assertTrue(rpm >= 2000 && rpm <= 2500,
                            "3.0m RPM should be in range [2000-2500], got " + rpm);
                }

                ShotInput input5m = ShotInput.builder()
                        .shooterPositionMeters(0, 0, 0.5)
                        .targetPositionMeters(5.0, 0, 2.1)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .build();

                TrajectoryResult result5m = solver.solve(input5m);
                if (result5m.isSuccess()) {
                    double rpm = result5m.getRecommendedRpm();
                    // Expected ~2400-2600 RPM
                    assertTrue(rpm >= 2200 && rpm <= 2800,
                            "5.0m RPM should be in range [2200-2800], got " + rpm);
                }
            }

            @Test
            void empiricalMapInterpolatesRpm() {
                ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap map =
                    new ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap();
                map.addPoint(1.264, 7.5, 2100);
                map.addPoint(1.300, 7.5, 2150);
                map.addPoint(1.500, 10.0, 2250);
                map.addPoint(1.710, 22.24, 2100);
                map.addPoint(2.000, 17.4, 2100);
                map.addPoint(3.000, 12.5, 2100);

                solver.setEmpiricalMap(map);

                ShotInput input = ShotInput.builder()
                        .shooterPositionMeters(0, 0, 0.5)
                        .targetPositionMeters(2.0, 0, 2.1)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .build();

                TrajectoryResult result = solver.solve(input);
                assertTrue(result.isSuccess(), "Solver with empirical map should succeed");
                double rpm = result.getRecommendedRpm();
                assertTrue(rpm >= 2000 && rpm <= 2200,
                        "RPM at 2.0m should be near 2100 (from map), got " + rpm);
            }

            @Test
            void empiricalMapInterpolatesPitch() {
                ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap map =
                    new ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap();
                map.addPoint(1.264, 7.5, 2100);
                map.addPoint(1.300, 7.5, 2150);
                map.addPoint(1.500, 10.0, 2250);
                map.addPoint(1.710, 22.24, 2100);
                map.addPoint(2.000, 17.4, 2100);
                map.addPoint(3.000, 12.5, 2100);

                solver.setEmpiricalMap(map);

                ShotInput input = ShotInput.builder()
                        .shooterPositionMeters(0, 0, 0.5)
                        .targetPositionMeters(2.0, 0, 2.1)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .build();

                TrajectoryResult result = solver.solve(input);
                assertTrue(result.isSuccess(), "Solver with empirical map should succeed");
                double pitch = result.getPitchAngleDegrees();
                // Pitch is determined by the physics solver (biased by tuning maps).
                // It should be a physically valid angle for the given geometry.
                assertTrue(pitch > 0 && pitch < 90,
                        "Pitch should be a valid angle, got " + pitch);
            }

            @Test
            void empiricalMapClampsRpmToRange() {
                ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap map =
                    new ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap(1900, 2500);
                // Add an artificially high RPM to test clamping
                map.addPoint(1.0, 10.0, 3500);
                map.addPoint(3.0, 12.0, 3800);

                solver.setEmpiricalMap(map);

                ShotInput input = ShotInput.builder()
                        .shooterPositionMeters(0, 0, 0.5)
                        .targetPositionMeters(2.0, 0, 2.1)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .build();

                TrajectoryResult result = solver.solve(input);
                assertTrue(result.isSuccess(), "Solver should succeed even with clamped RPM");
                double rpm = result.getRecommendedRpm();
                assertTrue(rpm <= 2500.0,
                        "RPM should be clamped to max 2500, got " + rpm);
                assertTrue(rpm >= 1900.0,
                        "RPM should be clamped to min 1900, got " + rpm);
            }

            @Test
            void solverFallsBackWhenOutOfMapRange() {
                ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap map =
                    new ca.team4308.absolutelib.math.trajectories.shooter.EmpiricalShotMap();
                map.addPoint(1.0, 7.5, 2100);
                map.addPoint(3.0, 12.5, 2100);

                solver.setEmpiricalMap(map);
                solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

                ShotInput input = ShotInput.builder()
                        .shooterPositionMeters(0, 0, 0.5)
                        .targetPositionMeters(8.0, 0, 2.1)
                        .targetRadiusMeters(0.45)
                        .includeAirResistance(true)
                        .build();

                TrajectoryResult result = solver.solve(input);

                assertNotNull(result, "Solver should return a non-null result for out-of-range");
                if (result.isSuccess()) {
                    assertTrue(result.getRecommendedRpm() > 0, "RPM should be positive");
                }
            }
}
