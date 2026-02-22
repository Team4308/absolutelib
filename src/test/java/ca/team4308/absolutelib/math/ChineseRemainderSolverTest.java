package ca.team4308.absolutelib.math;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;

public class ChineseRemainderSolverTest {

    // ── Core CRT Tests ──────────────────────────────────────────────────────

    @Test
    public void testSolvePairBasic() {
        // x ≡ 2 (mod 3), x ≡ 3 (mod 5) → x = 8, period = 15
        long[] result = ChineseRemainderSolver.solvePair(2, 3, 3, 5);
        assertNotNull(result);
        assertEquals(8, result[0]);
        assertEquals(15, result[1]);
    }

    @Test
    public void testSolvePairNoSolution() {
        // x ≡ 0 (mod 2), x ≡ 1 (mod 4) → no solution (both even-based but contradictory)
        long[] result = ChineseRemainderSolver.solvePair(0, 2, 1, 4);
        assertNull(result);
    }

    @Test
    public void testSolveMultipleConstraints() {
        // Classic CRT: x ≡ 2 (mod 3), x ≡ 3 (mod 5), x ≡ 2 (mod 7)
        // Solution: x = 23, period = 105
        List<ChineseRemainderSolver.Constraint> constraints = Arrays.asList(
            new ChineseRemainderSolver.Constraint("a", 2, 3),
            new ChineseRemainderSolver.Constraint("b", 3, 5),
            new ChineseRemainderSolver.Constraint("c", 2, 7)
        );
        ChineseRemainderSolver.Solution sol = ChineseRemainderSolver.solve(constraints);
        assertNotNull(sol);
        assertEquals(23, sol.getValue());
        assertEquals(105, sol.getPeriod());
        assertTrue(ChineseRemainderSolver.satisfiesAll(23, constraints));
    }

    @Test
    public void testSolveEmptyReturnsNull() {
        assertNull(ChineseRemainderSolver.solve(List.of()));
    }

    @Test
    public void testSolveSingleConstraint() {
        List<ChineseRemainderSolver.Constraint> constraints = List.of(
            new ChineseRemainderSolver.Constraint("enc", 100, 4096)
        );
        ChineseRemainderSolver.Solution sol = ChineseRemainderSolver.solve(constraints);
        assertNotNull(sol);
        assertEquals(100, sol.getValue());
        assertEquals(4096, sol.getPeriod());
    }

    @Test
    public void testSolutionNearest() {
        // x = 8 + 15k, nearest to 100 → 8 + 6*15 = 98
        List<ChineseRemainderSolver.Constraint> constraints = Arrays.asList(
            new ChineseRemainderSolver.Constraint("a", 2, 3),
            new ChineseRemainderSolver.Constraint("b", 3, 5)
        );
        ChineseRemainderSolver.Solution sol = ChineseRemainderSolver.solve(constraints);
        assertNotNull(sol);
        assertEquals(98, sol.nearest(100));
    }

    @Test
    public void testSolutionsInRange() {
        // x = 8 + 15k, range [0, 50] → 8, 23, 38
        List<ChineseRemainderSolver.Constraint> constraints = Arrays.asList(
            new ChineseRemainderSolver.Constraint("a", 2, 3),
            new ChineseRemainderSolver.Constraint("b", 3, 5)
        );
        ChineseRemainderSolver.Solution sol = ChineseRemainderSolver.solve(constraints);
        assertNotNull(sol);
        List<Long> inRange = sol.solutionsInRange(0, 50);
        assertEquals(List.of(8L, 23L, 38L), inRange);
    }

    // ── Turret Anti-Windup Tests ────────────────────────────────────────────

    @Test
    public void testDegreesToTicksAndBack() {
        assertEquals(1024, ChineseRemainderSolver.degreesToTicks(90.0, 4096), 0.001);
        assertEquals(2048, ChineseRemainderSolver.degreesToTicks(180.0, 4096), 0.001);
        assertEquals(90.0, ChineseRemainderSolver.ticksToDegrees(1024, 4096), 0.001);
    }

    @Test
    public void testShortestWrappedMoveForward() {
        // Current: 1000, target: 1500, ticksPerRev: 4096
        // Shortest move: +500 (forward)
        double move = ChineseRemainderSolver.shortestWrappedMove(1000, 1500, 4096);
        assertEquals(500.0, move, 0.001);
    }

    @Test
    public void testShortestWrappedMoveBackward() {
        // Current: 100, target: 3900, ticksPerRev: 4096
        // Forward: +3800, Backward: -296 → backward is shorter
        double move = ChineseRemainderSolver.shortestWrappedMove(100, 3900, 4096);
        assertEquals(-296.0, move, 0.001);
    }

    @Test
    public void testShortestWrappedMoveWraparound() {
        // Current: 3900, target: 100, ticksPerRev: 4096
        // Forward wrapping: +296, backward: -3800 → forward is shorter
        double move = ChineseRemainderSolver.shortestWrappedMove(3900, 100, 4096);
        assertEquals(296.0, move, 0.001);
    }

    @Test
    public void testClampToWindupLimitsInBounds() {
        // Current: 1000, target tick: 1500, limits: [-6144, 6144]
        // Shortest move is +500, dest = 1500 → in bounds
        double dest = ChineseRemainderSolver.clampToWindupLimits(1000, 1500, 4096, -6144, 6144);
        assertEquals(1500.0, dest, 0.001);
    }

    @Test
    public void testClampToWindupLimitsExceedingLimit() {
        // Current: 5500, target tick: 1000, ticksPerRev: 4096
        // Shortest move: -500 → dest 5000 (in bounds for [-6144, 6144])
        // But also possible: +3596 → dest 9096 (exceeds limit)
        double dest = ChineseRemainderSolver.clampToWindupLimits(5500, 1000, 4096, -6144, 6144);
        assertTrue(dest >= -6144 && dest <= 6144, "Destination should be within limits");
    }

    @Test
    public void testSafeTurretDestination() {
        // Target 90° on 4096 ticks/rev = tick 1024
        // Current at 0, limits ±6144
        double dest = ChineseRemainderSolver.safeTurretDestination(0, 90.0, 4096, -6144, 6144);
        assertEquals(1024.0, dest, 0.001);
    }

    @Test
    public void testSafeTurretDestinationPrefersShorterPath() {
        // Current at 0, target 350° = tick 3982.2
        // Forward: +3982, backward: -113.8 → should pick backward
        double dest = ChineseRemainderSolver.safeTurretDestination(0, 350.0, 4096, -6144, 6144);
        assertTrue(dest < 0, "Should rotate backward (negative) for 350° from 0");
    }

    // ── Utility Tests ───────────────────────────────────────────────────────

    @Test
    public void testSnapToResolution() {
        assertEquals(3500.0, ChineseRemainderSolver.snapToResolution(3487, 100), 0.001);
        assertEquals(45.5, ChineseRemainderSolver.snapToResolution(45.47, 0.1), 0.001);
    }

    @Test
    public void testGcdAndLcm() {
        assertEquals(6, ChineseRemainderSolver.gcd(12, 18));
        assertEquals(36, ChineseRemainderSolver.lcm(12, 18));
        assertEquals(1, ChineseRemainderSolver.gcd(17, 13));
        assertEquals(221, ChineseRemainderSolver.lcm(17, 13));
    }

    @Test
    public void testConstraintNormalizesNegativeRemainder() {
        ChineseRemainderSolver.Constraint c = new ChineseRemainderSolver.Constraint("test", -1, 5);
        assertEquals(4, c.getRemainder());
        assertTrue(c.isSatisfiedBy(4));
        assertTrue(c.isSatisfiedBy(9));
    }

    @Test
    public void testConstraintRejectsNonPositiveModulus() {
        assertThrows(IllegalArgumentException.class,
            () -> new ChineseRemainderSolver.Constraint("bad", 0, 0));
        assertThrows(IllegalArgumentException.class,
            () -> new ChineseRemainderSolver.Constraint("bad", 0, -5));
    }
}
