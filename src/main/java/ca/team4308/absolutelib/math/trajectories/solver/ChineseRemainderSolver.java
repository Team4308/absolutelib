package ca.team4308.absolutelib.math.trajectories.solver;

import java.util.ArrayList;
import java.util.List;

/**
 * Chinese Remainder Theorem (CRT) solver utility.
 * 
 * Used to find solutions that satisfy multiple discrete modular constraints.
 */
public class ChineseRemainderSolver {
    
    /**
     * A modular constraint of the form: x ≡ remainder (mod modulus)
     */
    public static class ModularConstraint {
        public final String name;
        public final long remainder;
        public final long modulus;
        
        public ModularConstraint(String name, long remainder, long modulus) {
            this.name = name;
            this.remainder = remainder % modulus;
            this.modulus = modulus;
        }
        
        public boolean isSatisfiedBy(long value) {
            return value % modulus == remainder;
        }
        
        @Override
        public String toString() {
            return String.format("%s: x ≡ %d (mod %d)", name, remainder, modulus);
        }
    }
    
    /**
     * A discrete solution found by the CRT solver.
     */
    public static class CRTSolution {
        public final long value;
        public final long period;  // Solutions repeat every 'period' units
        public final List<ModularConstraint> satisfiedConstraints;
        
        public CRTSolution(long value, long period, List<ModularConstraint> satisfiedConstraints) {
            this.value = value;
            this.period = period;
            this.satisfiedConstraints = satisfiedConstraints;
        }
        
        /**
         * Gets all solutions within a range.
         */
        public List<Long> getSolutionsInRange(long minValue, long maxValue) {
            List<Long> solutions = new ArrayList<>();
            
            // Find first solution >= minValue
            long firstSolution = value;
            if (firstSolution < minValue) {
                long offset = (minValue - firstSolution + period - 1) / period;
                firstSolution += offset * period;
            }
            
            // Enumerate solutions up to maxValue
            for (long sol = firstSolution; sol <= maxValue; sol += period) {
                solutions.add(sol);
            }
            
            return solutions;
        }
        
        @Override
        public String toString() {
            return String.format("x = %d + %dk (for integer k)", value, period);
        }
    }
    
    /**
     * Solves the classical CRT for two congruences.
     * Finds x such that: x ≡ a1 (mod m1) and x ≡ a2 (mod m2)
     * 
     * @param a1 First remainder
     * @param m1 First modulus
     * @param a2 Second remainder
     * @param m2 Second modulus
     * @return Solution where x = result[0] and period = result[1], or null if no solution
     */
    public static long[] solveTwoCongruences(long a1, long m1, long a2, long m2) {
        // Extended Euclidean algorithm to find GCD and coefficients
        long[] gcdResult = extendedGcd(m1, m2);
        long gcd = gcdResult[0];
        long x = gcdResult[1];
        // long y = gcdResult[2]; // Not needed here
        
        // Check if solution exists
        if ((a2 - a1) % gcd != 0) {
            return null;  // No solution
        }
        
        // LCM of moduli
        long lcm = m1 / gcd * m2;
        
        // Solution: x ≡ a1 + m1 * x * ((a2 - a1) / gcd) (mod lcm)
        long solution = a1 + m1 * x * ((a2 - a1) / gcd);
        
        // Normalize to positive
        solution = ((solution % lcm) + lcm) % lcm;
        
        return new long[]{solution, lcm};
    }
    
    /**
     * Solves CRT for multiple congruences.
     * 
     * @param constraints List of modular constraints
     * @return CRT solution, or null if no solution exists
     */
    public static CRTSolution solveMultiple(List<ModularConstraint> constraints) {
        if (constraints.isEmpty()) {
            return null;
        }
        
        if (constraints.size() == 1) {
            ModularConstraint c = constraints.get(0);
            return new CRTSolution(c.remainder, c.modulus, constraints);
        }
        
        // Start with first constraint
        long currentRemainder = constraints.get(0).remainder;
        long currentModulus = constraints.get(0).modulus;
        
        // Iteratively combine with remaining constraints
        for (int i = 1; i < constraints.size(); i++) {
            ModularConstraint next = constraints.get(i);
            long[] result = solveTwoCongruences(
                currentRemainder, currentModulus,
                next.remainder, next.modulus
            );
            
            if (result == null) {
                return null;  // No solution
            }
            
            currentRemainder = result[0];
            currentModulus = result[1];
        }
        
        return new CRTSolution(currentRemainder, currentModulus, constraints);
    }

    /**
     * Extended Euclidean algorithm.
     * Returns [gcd, x, y] such that gcd = a*x + b*y
     */
    private static long[] extendedGcd(long a, long b) {
        if (b == 0) {
            return new long[]{a, 1, 0};
        }
        
        long[] result = extendedGcd(b, a % b);
        long gcd = result[0];
        long x = result[2];
        long y = result[1] - (a / b) * result[2];
        
        return new long[]{gcd, x, y};
    }

    /**
     * Calculates GCD of two numbers.
     */
    public static long gcd(long a, long b) {
        while (b != 0) {
            long temp = b;
            b = a % b;
            a = temp;
        }
        return a;
    }
    
    /**
     * Calculates LCM of two numbers.
     */
    public static long lcm(long a, long b) {
        return a / gcd(a, b) * b;
    }
    
    /**
     * Checks if a value satisfies all constraints.
     */
    public static boolean satisfiesAll(long value, List<ModularConstraint> constraints) {
        for (ModularConstraint c : constraints) {
            if (!c.isSatisfiedBy(value)) {
                return false;
            }
        }
        return true;
    }
}
