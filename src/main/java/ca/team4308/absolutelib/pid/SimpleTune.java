package ca.team4308.absolutelib.pid;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;

/**
 * A Relay-based (Bang-Bang) Auto-Tuner for PID controllers. Uses the
 * Åström–Hägglund Relay method to estimate Ultimate Gain (Ku) and Ultimate
 * Period (Tu). Read the NI Documation on auto PID tuning its a really cool read
 */
public class SimpleTune {

    public enum TuningRule {
        ZIEGLER_NICHOLS,
        TYREUS_LUYBEN,
        P_ONLY,
        PI_ONLY,
        NO_OVERSHOOT,
        COHEN_COON,
        AMIGOF
    }

    /**
     * A Coordinate Descent (Twiddle) optimizer that iteratively refines PID
     * constants.
     * <p>
     * Usage: 1. Initialize PIDOptimizer with starting constants (e.g. from
     * Ziegler-Nichols). 2. Run a test cycle (e.g. move mechanism to setpoint).
     * 3. Calculate a score (e.g. Integral Squared Error). 4. Call
     * {@code tune(score)} to get the next set of constants to try. 5. Repeat
     * until satisfied.
     * </p>
     */
    public static class PIDOptimizer {

        private double[] p = new double[3]; // kP, kI, kD
        private double[] dp = new double[3]; // Step sizes
        private double bestError = Double.MAX_VALUE;
        private int paramIdx = 0;
        private int state = 0;
        private boolean firstRun = true;
        private int precision = 5;

        public PIDOptimizer(double startP, double startI, double startD) {
            this(startP, startI, startD, 5);
        }

        /**
         * @param startP Initial kP
         * @param startI Initial kI
         * @param startD Initial kD
         * @param precision Decimal places to solve for (determines min step
         * size).
         */
        public PIDOptimizer(double startP, double startI, double startD, int precision) {
            this.precision = precision;
            this.p[0] = startP;
            this.p[1] = startI;
            this.p[2] = startD;

            double minStep = Math.pow(10, -precision);
            this.dp[0] = startP != 0 ? Math.abs(startP) * 0.1 : minStep;
            this.dp[1] = startI != 0 ? Math.abs(startI) * 0.1 : minStep;
            this.dp[2] = startD != 0 ? Math.abs(startD) * 0.1 : minStep;
        }

        /**
         * Processes the error from the last run and returns the next PID
         * constants to try.
         *
         * @param errorScore The total error (cost) of the last run (lower is
         * better).
         * @return The new PIDResult to test.
         */
        public PIDResult tune(double errorScore) {
            if (firstRun) {
                bestError = errorScore;
                firstRun = false;
                p[paramIdx] += dp[paramIdx];
                state = 1;
                return currentPID();
            }

            switch (state) {
                case 1:
                    if (errorScore < bestError) {
                        bestError = errorScore;
                        dp[paramIdx] *= 1.1; // increase step size

                        paramIdx = (paramIdx + 1) % 3;
                        p[paramIdx] += dp[paramIdx];
                        state = 1;
                    } else {
                        //subtracting
                        p[paramIdx] -= 2 * dp[paramIdx]; // subtract
                        state = 2;
                    }
                    break;

                case 2: //  subtracting dp[i]
                    if (errorScore < bestError) {
                        bestError = errorScore;
                        dp[paramIdx] *= 1.1;
                    } else {
                        //  restore and decrease step size
                        p[paramIdx] += dp[paramIdx];
                        dp[paramIdx] *= 0.9;
                    }

                    // Move to next parameter
                    paramIdx = (paramIdx + 1) % 3;
                    p[paramIdx] += dp[paramIdx];
                    state = 1;
                    break;
            }

            return currentPID();
        }

        public PIDResult currentPID() {
            return new PIDResult(round(p[0]), round(p[1]), round(p[2]));
        }

        private double round(double value) {
            double scale = Math.pow(10, precision);
            return Math.round(value * scale) / scale;
        }

        public boolean isConverged(double tolerance) {
            return (dp[0] + dp[1] + dp[2]) < tolerance;
        }
    }

    public static class PIDResult {

        public double kP;
        public double kI;
        public double kD;
        public double kS;
        public double kV;
        public double kA;
        public double kG;

        public PIDResult(double p, double i, double d) {
            this(p, i, d, 0, 0, 0, 0);
        }

        public PIDResult(double p, double i, double d, double s, double v, double a, double g) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
            this.kS = s;
            this.kV = v;
            this.kA = a;
            this.kG = g;
        }

        @Override
        public String toString() {
            return String.format("P: %.5f, I: %.5f, D: %.5f, S: %.5f, V: %.5f, A: %.5f, G: %.5f", kP, kI, kD, kS, kV, kA, kG);
        }
    }

    private final double relayOutput;
    private final double setpoint;
    private final double hysteresis;

    private boolean isRunning = false;
    private double startTime = 0;

    private final List<Double> periods = new ArrayList<>();
    private final List<Double> amplitudes = new ArrayList<>();

    private boolean lastOutputPositive = false;
    private double cycleMin = Double.MAX_VALUE;
    private double cycleMax = -Double.MAX_VALUE;
    private double lastSwitchTime = 0;

    private int requiredCycles = 5;
    private int precision = 5;  // Lowkey idk

    // Feedforward 
    private double kS_est = 0.0;
    private double kV_est = 0.0;
    private double kG_est = 0.0;

    public SimpleTune(double setpoint, double relayOutput, double hysteresis) {
        this.setpoint = setpoint;
        this.relayOutput = relayOutput;
        this.hysteresis = hysteresis;
    }

    /**
     * Sets the number of decimal places to solve for.
     *
     * @param decimals The number of decimal places (e.g. 5). lowkey anything
     * beyond 4 is cooked and will take forever
     */
    public void setPrecision(int decimals) {
        this.precision = decimals;
    }

    private double round(double value) {
        double scale = Math.pow(10, precision);
        return Math.round(value * scale) / scale;
    }

    public void start() {
        isRunning = true;
        startTime = Timer.getFPGATimestamp();
        lastSwitchTime = startTime;
        periods.clear();
        amplitudes.clear();
        cycleMin = Double.MAX_VALUE;
        cycleMax = -Double.MAX_VALUE;
    }

    public double update(double measurement) {
        if (!isRunning) {
            return 0.0;
        }

        double error = setpoint - measurement;
        double now = Timer.getFPGATimestamp();

        if (measurement > cycleMax) {
            cycleMax = measurement;
        }
        if (measurement < cycleMin) {
            cycleMin = measurement;
        }

        double output = 0.0;

        if (Math.abs(error) < hysteresis) {
            output = lastOutputPositive ? relayOutput : -relayOutput;
        } else if (error > 0) {
            output = relayOutput;
        } else {
            output = -relayOutput;
        }

        boolean currentOutputPositive = output > 0;

        if (currentOutputPositive != lastOutputPositive) {
            if (currentOutputPositive) {
                double cycleTime = now - lastSwitchTime;

                if (periods.size() > 0 || (now - startTime) > 0.5) {
                    periods.add(cycleTime);
                    double amplitude = (cycleMax - cycleMin) / 2.0;
                    amplitudes.add(amplitude);
                }

                lastSwitchTime = now;
                cycleMin = measurement;
                cycleMax = measurement;
            }
        }

        lastOutputPositive = currentOutputPositive;
        return output;
    }

    public boolean isFinished() {
        return periods.size() >= requiredCycles;
    }

    /**
     * Estimates kS (Static Friction) by slowly ramping voltage until movement
     * is detected. This is a blocking operation if run in a loop, but designed
     * to be called periodically.
     *
     * @param currentVelocity The current velocity of the mechanism.
     * @param voltageStep The amount to increase voltage per call (e.g., 0.05).
     * @param velocityThreshold The velocity threshold to consider "moving"
     * (e.g., 0.01).
     * @return The voltage to apply. Returns 0 when finished.
     */
    public double determineKS(double currentVelocity, double voltageStep, double velocityThreshold) {
        if (Math.abs(currentVelocity) > velocityThreshold) {
            kS_est = kS_est; // Store current voltage as kS
            return 0.0; // Finished
        }
        kS_est += voltageStep;
        return kS_est;
    }

    /**
     * Estimates kG (Gravity Feedforward) by adjusting voltage until the
     * mechanism holds position. Call this periodically while the mechanism is
     * in a position where gravity acts on it (e.g., horizontal arm).
     *
     * @param currentVelocity The current velocity of the mechanism.
     * @param voltageStep The amount to adjust voltage per call (e.g., 0.01).
     * @param velocityThreshold The velocity threshold to consider "holding"
     * (e.g., 0.01).
     * @return The voltage to apply (which is the current estimate of kG).
     */
    public double determineKG(double currentVelocity, double voltageStep, double velocityThreshold) {
        if (currentVelocity < -velocityThreshold) {
            // Falling, need more voltage
            kG_est += voltageStep;
        } else if (currentVelocity > velocityThreshold) {
            // Rising, need less voltage
            kG_est -= voltageStep;
        }
        // If within threshold, we keep the current kG_est
        return kG_est;
    }

    /**
     * Estimates kV (Velocity Feedforward) assuming the mechanism is at
     * steady-state velocity. kV = AppliedVoltage / Velocity.
     */
    public void estimateKV(double appliedVoltage, double currentVelocity) {
        if (Math.abs(currentVelocity) > 1e-3) {
            kV_est = appliedVoltage / currentVelocity;
        }
    }

    public PIDResult calculateConstants(TuningRule rule) {
        if (periods.isEmpty() || amplitudes.isEmpty()) {
            return new PIDResult(0, 0, 0);
        }

        double Tu = periods.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        double A = amplitudes.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        double Ku = (4.0 * relayOutput) / (Math.PI * A);

        double kp = 0, ki = 0, kd = 0;

        switch (rule) {
            case ZIEGLER_NICHOLS:
                kp = 0.6 * Ku;
                ki = 1.2 * Ku / Tu;
                kd = 0.075 * Ku * Tu;
                break;
            case TYREUS_LUYBEN:
                kp = 0.45 * Ku;
                ki = kp / (2.2 * Tu);
                kd = kp * (Tu / 6.3);
                break;
            case PI_ONLY:
                kp = 0.45 * Ku;
                ki = 0.54 * Ku / Tu;
                kd = 0;
                break;
            case P_ONLY:
                kp = 0.5 * Ku;
                ki = 0;
                kd = 0;
                break;
            case NO_OVERSHOOT:
                kp = 0.2 * Ku;
                ki = 0.4 * Ku / Tu;
                kd = 0.066 * Ku * Tu;
                break;
            case COHEN_COON:
                kp = 0.66 * Ku;
                ki = 2.5 * (kp / Tu);
                kd = 0.37 * (kp * Tu);
                break;
            case AMIGOF:
                kp = 0.25 * Ku;
                ki = 0.25 * Ku / Tu;
                kd = 0.1 * Ku * Tu;
                break;
        }

        return new PIDResult(round(kp), round(ki), round(kd), round(kS_est), round(kV_est), 0.0, round(kG_est));
    }

    public String getStatus() {
        return String.format("Cycles: %d/%d, Last Period: %.3fs", periods.size(), requiredCycles,
                periods.isEmpty() ? 0.0 : periods.get(periods.size() - 1));
    }
}
