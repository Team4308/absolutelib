package ca.team4308.absolutelib.math;

import java.util.ArrayList;
import java.util.List;

/**
 * Generic Chinese Remainder Theorem (CRT) solver with turret anti-windup utilities.
 *
 * <p>The CRT finds a value {@code x} that simultaneously satisfies multiple modular
 * congruences of the form {@code x ≡ rᵢ (mod mᵢ)}. This is useful in FRC for:
 * <ul>
 *   <li><b>Turret anti-windup:</b> finding the shortest rotation to a target heading
 *       that lands on a valid encoder tick boundary without exceeding cable limits.</li>
 *   <li><b>Discrete shot snapping:</b> mapping continuous RPM/angle solutions to
 *       achievable hardware step values.</li>
 *   <li><b>Gear-ratio alignment:</b> finding positions where multiple geared axes
 *       align simultaneously.</li>
 * </ul>
 *
 * <p><b>Turret Anti-Windup Example</b></p>
 * <pre>{@code
 * // Turret: 4096 ticks/rev, currently at tick 1200, target heading = 270°
 * // Cable limit: ±1.5 rotations (±6144 ticks from home)
 * double targetDegrees = 270.0;
 * double currentTicks = 1200;
 * int ticksPerRev = 4096;
 *
 * double targetTick = ChineseRemainderSolver.degreesToTicks(targetDegrees, ticksPerRev);
 * double shortestMove = ChineseRemainderSolver.shortestWrappedMove(currentTicks, targetTick, ticksPerRev);
 * double destination = currentTicks + shortestMove;
 *
 * // Clamp to cable limits
 * double safeDest = ChineseRemainderSolver.clampToWindupLimits(
 *         currentTicks, targetTick, ticksPerRev, -6144, 6144);
 * }</pre>
 *
 * @see DoubleUtils
 */
public final class ChineseRemainderSolver {

    private ChineseRemainderSolver() {}

    // ── Modular Constraint API ──────────────────────────────────────────────

    /**
     * A modular constraint of the form: {@code x ≡ remainder (mod modulus)}.
     */
    public static class Constraint {
        private final String name;
        private final long remainder;
        private final long modulus;

        /**
         * Creates a named modular constraint.
         *
         * @param name      human-readable label (e.g. "encoder", "gear mesh")
         * @param remainder the required remainder
         * @param modulus   the modulus (must be positive)
         */
        public Constraint(String name, long remainder, long modulus) {
            if (modulus <= 0) {
                throw new IllegalArgumentException("Modulus must be positive, got " + modulus);
            }
            this.name = name;
            this.remainder = ((remainder % modulus) + modulus) % modulus;
            this.modulus = modulus;
        }

        /** Returns the constraint name. */
        public String getName() { return name; }

        /** Returns the normalized remainder (always in {@code [0, modulus)}). */
        public long getRemainder() { return remainder; }

        /** Returns the modulus. */
        public long getModulus() { return modulus; }

        /** Returns {@code true} if the given value satisfies this constraint. */
        public boolean isSatisfiedBy(long value) {
            return ((value % modulus) + modulus) % modulus == remainder;
        }

        @Override
        public String toString() {
            return String.format("%s: x ≡ %d (mod %d)", name, remainder, modulus);
        }
    }

    /**
     * Result of a CRT solve: the smallest non-negative solution and its period.
     * All solutions are of the form {@code value + k * period} for any integer {@code k}.
     */
    public static class Solution {
        private final long value;
        private final long period;
        private final List<Constraint> constraints;

        /**
         * @param value       smallest non-negative solution
         * @param period      distance between consecutive solutions (LCM of moduli)
         * @param constraints the constraints this solution satisfies
         */
        public Solution(long value, long period, List<Constraint> constraints) {
            this.value = value;
            this.period = period;
            this.constraints = constraints;
        }

        /** Returns the smallest non-negative solution. */
        public long getValue() { return value; }

        /** Returns the period — solutions repeat every this many units. */
        public long getPeriod() { return period; }

        /** Returns the constraints this solution satisfies. */
        public List<Constraint> getConstraints() { return constraints; }

        /**
         * Enumerates all solutions in the range {@code [min, max]}.
         *
         * @param min inclusive lower bound
         * @param max inclusive upper bound
         * @return list of all valid solutions in range
         */
        public List<Long> solutionsInRange(long min, long max) {
            List<Long> results = new ArrayList<>();
            long first = value;
            if (first < min) {
                long offset = (min - first + period - 1) / period;
                first += offset * period;
            } else if (first > max) {
                return results;
            }
            for (long s = first; s <= max; s += period) {
                results.add(s);
            }
            return results;
        }

        /**
         * Returns the solution nearest to a target value.
         *
         * @param target the value to get close to
         * @return the solution {@code value + k*period} closest to target
         */
        public long nearest(long target) {
            long diff = target - value;
            long k = Math.round((double) diff / period);
            return value + k * period;
        }

        @Override
        public String toString() {
            return String.format("x = %d + %dk", value, period);
        }
    }

    // ── Legacy Aliases ────────────────────────────────────────────────────

    /**
     * Legacy alias for {@link Constraint}.
     * Kept for backward compatibility with code using the original name.
     */
    public static class ModularConstraint extends Constraint {
        /** Creates a named modular constraint. */
        public ModularConstraint(String name, long remainder, long modulus) {
            super(name, remainder, modulus);
        }
    }

    /**
     * Legacy alias for {@link Solution}.
     * Kept for backward compatibility with code using the original name.
     */
    public static class CRTSolution extends Solution {
        /** Creates a CRT solution. */
        public CRTSolution(long value, long period, List<Constraint> constraints) {
            super(value, period, constraints);
        }

        /**
         * Gets all solutions within a range.
         * Legacy alias for {@link Solution#solutionsInRange(long, long)}.
         */
        public List<Long> getSolutionsInRange(long minValue, long maxValue) {
            return solutionsInRange(minValue, maxValue);
        }
    }

    // ── Core CRT Solver ─────────────────────────────────────────────────────

    /**
     * Solves two congruences: {@code x ≡ a1 (mod m1)} and {@code x ≡ a2 (mod m2)}.
     *
     * @return {@code [solution, lcm]} or {@code null} if no solution exists
     *         (i.e., gcd(m1,m2) does not divide a2−a1)
     */
    public static long[] solvePair(long a1, long m1, long a2, long m2) {
        long[] ext = extendedGcd(m1, m2);
        long g = ext[0], p = ext[1];
        if ((a2 - a1) % g != 0) {
            return null;
        }
        long lcm = m1 / g * m2;
        long solution = a1 + m1 * p % lcm * ((a2 - a1) / g) % lcm;
        solution = ((solution % lcm) + lcm) % lcm;
        return new long[]{solution, lcm};
    }

    /**
     * Solves a system of modular constraints using iterative pairwise CRT.
     *
     * @param constraints the constraints to satisfy simultaneously
     * @return the combined solution, or {@code null} if the system is inconsistent
     */
    public static Solution solve(List<Constraint> constraints) {
        if (constraints == null || constraints.isEmpty()) {
            return null;
        }
        if (constraints.size() == 1) {
            Constraint c = constraints.get(0);
            return new Solution(c.remainder, c.modulus, constraints);
        }
        long r = constraints.get(0).remainder;
        long m = constraints.get(0).modulus;
        for (int i = 1; i < constraints.size(); i++) {
            Constraint next = constraints.get(i);
            long[] pair = solvePair(r, m, next.remainder, next.modulus);
            if (pair == null) {
                return null;
            }
            r = pair[0];
            m = pair[1];
        }
        return new Solution(r, m, constraints);
    }

    /**
     * Checks whether a value satisfies all constraints.
     *
     * @param value       the value to test
     * @param constraints constraints to check
     * @return {@code true} if every constraint is satisfied
     */
    public static boolean satisfiesAll(long value, List<Constraint> constraints) {
        for (Constraint c : constraints) {
            if (!c.isSatisfiedBy(value)) {
                return false;
            }
        }
        return true;
    }

    // ── Turret Anti-Windup Utilities ────────────────────────────────────────

    /**
     * Converts a heading in degrees to an encoder tick value.
     *
     * @param degrees     heading in degrees (0–360)
     * @param ticksPerRev encoder ticks per full revolution
     * @return the corresponding tick value (fractional)
     */
    public static double degreesToTicks(double degrees, int ticksPerRev) {
        return (degrees / 360.0) * ticksPerRev;
    }

    /**
     * Converts encoder ticks to degrees.
     *
     * @param ticks       encoder tick value
     * @param ticksPerRev encoder ticks per full revolution
     * @return heading in degrees
     */
    public static double ticksToDegrees(double ticks, int ticksPerRev) {
        return (ticks / ticksPerRev) * 360.0;
    }

    /**
     * Computes the shortest signed move (in ticks) from the current position
     * to the target tick, wrapping around one revolution.
     *
     * <p>Returns a value in {@code (-ticksPerRev/2, ticksPerRev/2]}, representing
     * the shortest-path direction. Positive = clockwise, negative = counter-clockwise
     * (or vice versa depending on encoder convention).
     *
     * @param currentTicks current encoder position
     * @param targetTick   desired encoder position (mod one revolution)
     * @param ticksPerRev  ticks per full revolution
     * @return shortest signed move in ticks
     */
    public static double shortestWrappedMove(double currentTicks, double targetTick, int ticksPerRev) {
        double diff = targetTick - currentTicks;
        diff = ((diff % ticksPerRev) + ticksPerRev) % ticksPerRev;
        if (diff > ticksPerRev / 2.0) {
            diff -= ticksPerRev;
        }
        return diff;
    }

    /**
     * Finds the destination tick closest to the current position that corresponds
     * to the target heading, while staying within absolute windup limits.
     *
     * <p>This is the primary anti-windup method. It enumerates all equivalent
     * positions for the target heading (spaced one revolution apart) and picks
     * the one closest to the current position that doesn't exceed the cable limits.
     *
     * <p>If no in-bounds solution exists, returns the clamped limit in the
     * direction of the shortest move.
     *
     * @param currentTicks current absolute encoder position
     * @param targetTick   target position modulo one revolution (0 to ticksPerRev)
     * @param ticksPerRev  ticks per full revolution
     * @param minTicks     minimum absolute tick limit (negative = CCW limit)
     * @param maxTicks     maximum absolute tick limit (positive = CW limit)
     * @return the safe destination tick
     */
    public static double clampToWindupLimits(double currentTicks, double targetTick,
                                              int ticksPerRev, double minTicks, double maxTicks) {
        double shortestMove = shortestWrappedMove(currentTicks, targetTick, ticksPerRev);
        double idealDest = currentTicks + shortestMove;

        if (idealDest >= minTicks && idealDest <= maxTicks) {
            return idealDest;
        }

        double bestDest = Double.NaN;
        double bestDistance = Double.MAX_VALUE;

        double base = ((targetTick % ticksPerRev) + ticksPerRev) % ticksPerRev;
        double start = minTicks + ((base - minTicks) % ticksPerRev + ticksPerRev) % ticksPerRev;

        for (double candidate = start; candidate <= maxTicks; candidate += ticksPerRev) {
            if (candidate < minTicks) continue;
            double dist = Math.abs(candidate - currentTicks);
            if (dist < bestDistance) {
                bestDistance = dist;
                bestDest = candidate;
            }
        }

        if (Double.isNaN(bestDest)) {
            return shortestMove > 0 ? maxTicks : minTicks;
        }
        return bestDest;
    }

    /**
     * Convenience method: given a target heading in degrees and current absolute
     * tick position, returns the safe destination tick.
     *
     * @param currentTicks   current absolute encoder position
     * @param targetDegrees  desired heading in degrees
     * @param ticksPerRev    ticks per full revolution
     * @param minTicks       minimum absolute tick limit
     * @param maxTicks       maximum absolute tick limit
     * @return safe destination tick within cable limits
     */
    public static double safeTurretDestination(double currentTicks, double targetDegrees,
                                                int ticksPerRev, double minTicks, double maxTicks) {
        double targetTick = degreesToTicks(targetDegrees, ticksPerRev);
        return clampToWindupLimits(currentTicks, targetTick, ticksPerRev, minTicks, maxTicks);
    }

    /**
     * Snaps a continuous value to the nearest discrete step.
     * Useful for rounding RPM/angle to hardware-achievable values.
     *
     * @param value      the continuous value
     * @param resolution the step size (e.g., 1.0 RPM, 0.1°)
     * @return the nearest multiple of resolution
     */
    public static double snapToResolution(double value, double resolution) {
        return Math.round(value / resolution) * resolution;
    }

    // ── Number Theory Utilities ─────────────────────────────────────────────

    /**
     * Computes the greatest common divisor of two numbers.
     *
     * @param a first value
     * @param b second value
     * @return gcd(a, b)
     */
    public static long gcd(long a, long b) {
        a = Math.abs(a);
        b = Math.abs(b);
        while (b != 0) {
            long t = b;
            b = a % b;
            a = t;
        }
        return a;
    }

    /**
     * Computes the least common multiple of two numbers.
     *
     * @param a first value
     * @param b second value
     * @return lcm(a, b)
     */
    public static long lcm(long a, long b) {
        if (a == 0 || b == 0) return 0;
        return Math.abs(a) / gcd(a, b) * Math.abs(b);
    }

    /**
     * Extended Euclidean algorithm.
     * Returns {@code [gcd, x, y]} such that {@code a*x + b*y = gcd(a,b)}.
     *
     * @param a first value
     * @param b second value
     * @return array {@code [gcd, x, y]}
     */
    public static long[] extendedGcd(long a, long b) {
        if (b == 0) {
            return new long[]{a, 1, 0};
        }
        long[] r = extendedGcd(b, a % b);
        return new long[]{r[0], r[2], r[1] - (a / b) * r[2]};
    }
}
