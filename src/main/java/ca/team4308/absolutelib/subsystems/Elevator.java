package ca.team4308.absolutelib.subsystems;

import java.util.Arrays;
import java.util.List;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;

/**
 * Elevator helper with manual, position, and holding modes.
 * Uses {@link EncoderWrapper} for position and {@link MotorWrapper} for motor control.
 */
public class Elevator extends AbsoluteSubsystem {
    /** Invoke once to allow subclasses to do one-time setup. */
    @Override
    public final void initialize() { onInitialize(); }
    /** Hook: one-time initialization. */
    @Override
    protected void onInitialize() {}
    /** Default counts-per-revolution used for the CANCoder convenience constructor. */
    public static final double DEFAULT_CANCODER_CPR = 4028.0;

    /** Operating mode for the elevator. */
    public enum Mode { IDLE, MANUAL, POSITION, HOLDING }

    protected Mode mode = Mode.IDLE;
    protected double targetPositionMeters = 0.0;
    protected double manualPercentOutput = 0.0;

    private final MotorWrapper leaderMotor;
    private final List<MotorWrapper> followerMotors;
    private EncoderWrapper encoder;
    private final ElevatorConfig config;

    private final PIDController positionPid = new PIDController(0.0, 0.0, 0.0);
    private final PIDController holdPid = new PIDController(0.0, 0.0, 0.0);
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
    private double desiredVelocity = 0.0;
    private double desiredAcceleration = 0.0;
    private double nominalVoltage = 12.0;

    private final java.util.List<java.util.function.BiFunction<Double, Double, Double>> outputAugmentors = new java.util.ArrayList<>();


    /**
     * Backward-compatible convenience constructor using a CTRE CANCoder as the sensor.
     *
     * @param leaderMotor         configured leader motor
     * @param motorConfig         optional motor config applied to leader and followers (null to skip)
     * @param encoderCanId        CAN id of the CANCoder
     * @param gearRatio           sensor rotations per mechanism rotation
     * @param drumDiameterMeters  drum diameter (meters) for linear conversion
     * @param maxHeightMeters     top soft limit (meters)
     * @param minHeightMeters     bottom soft limit (meters)
     * @param maxVelocityMetersPerSec  nominal max velocity for planning/telemetry (meters/sec)
     * @param maxAccelerationMetersPerSecSq nominal max accel for planning/telemetry (meters/sec^2)
     * @param toleranceMeters     position tolerance for atTarget()/HOLDING (meters)
     * @param followers           zero or more follower motors (matched to leader)
     */
    public Elevator(
            MotorWrapper leaderMotor,
            MotorConfig motorConfig,
            int encoderCanId,
            double gearRatio,
            double drumDiameterMeters,
            double maxHeightMeters,
            double minHeightMeters,
            double maxVelocityMetersPerSec,
            double maxAccelerationMetersPerSecSq,
            double toleranceMeters,
            MotorWrapper... followers) {
        this(
            leaderMotor,
            motorConfig,
            EncoderWrapper.canCoder(encoderCanId, gearRatio, DEFAULT_CANCODER_CPR, drumDiameterMeters),
            ElevatorConfig.builder()
                .minHeightMeters(minHeightMeters)
                .maxHeightMeters(maxHeightMeters)
                .gearRatio(gearRatio)
                .drumDiameterMeters(drumDiameterMeters)
                .maxVelocityMetersPerSec(maxVelocityMetersPerSec)
                .maxAccelerationMetersPerSecSq(maxAccelerationMetersPerSecSq)
                .toleranceMeters(toleranceMeters)
                .build(),
            followers
        );
    }

    /**
     * Preferred constructor using an encoder abstraction and explicit configuration.
     * No overridable methods are invoked during construction.
     *
     * @param leaderMotor  configured leader motor
     * @param motorConfig  optional motor config applied to leader and followers (null to skip)
     * @param encoder      elevator position source
     * @param config       geometry and constraints
     * @param followers    zero or more follower motors (matched to leader)
     */
    public Elevator(MotorWrapper leaderMotor,
                    MotorConfig motorConfig,
                    EncoderWrapper encoder,
                    ElevatorConfig config,
                    MotorWrapper... followers) {
        super();
        this.leaderMotor = leaderMotor;
        this.followerMotors = Arrays.asList(followers);
        this.encoder = encoder;
        this.config = config;
        if (motorConfig != null) {
            this.leaderMotor.applyMotorConfig(motorConfig);
            for (MotorWrapper follower : this.followerMotors) {
                this.leaderMotor.addFollower(follower);
                follower.applyMotorConfig(motorConfig);
            }
        } else {
            for (MotorWrapper follower : this.followerMotors) {
                this.leaderMotor.addFollower(follower);
            }
        }
        double startPos = this.encoder.getPositionMeters();
        this.targetPositionMeters = DoubleUtils.clamp(startPos, config.minHeightMeters, config.maxHeightMeters);
        this.mode = Mode.HOLDING;
    }


    /**
     * Set manual/open-loop control percent output and switch to MANUAL mode.
     * Clamps to configured output limits.
     */
    public final void setManualPercent(double percent) {
        setMode(Mode.MANUAL);
        manualPercentOutput = clampPercent(percent);
    }

    /**
     * Set the target height in meters and enter POSITION mode.
     * The target is clamped to [minHeightMeters, maxHeightMeters].
     */
    public final void setPosition(double meters) {
        targetPositionMeters = DoubleUtils.clamp(meters, config.minHeightMeters, config.maxHeightMeters);
        setMode(Mode.POSITION);
        // Reset position PID to avoid integral windup on new target
        positionPid.reset();
    }

    /**
     * Stop the elevator (percent output = 0) and enter IDLE mode.
     * Triggers {@link #onStop()} after the mode change.
     */
    @Override
    public final void stop() {
        manualPercentOutput = 0.0;
        applyPercentOutput(0.0);
        setMode(Mode.IDLE);
        onStop();
    }

    /** Current position in meters from the {@link EncoderWrapper}. */
    public final double getCurrentPosition() { return encoder.getPositionMeters(); }
    /** Current operating {@link Mode}. */
    public final Mode getMode() { return mode; }
    /** Current position target in meters. */
    public final double getTargetPosition() { return targetPositionMeters; }
    /** True when the current position is within the configured tolerance of the target. */
    public final boolean atTarget() { return Math.abs(getCurrentPosition() - targetPositionMeters) <= config.toleranceMeters; }
    /** Swap the encoder instance at runtime (useful for redundancy or calibration). */
    public final void setEncoder(EncoderWrapper newEncoder) { this.encoder = newEncoder; }


    /**
     * Main update loop: calls pre-hook, runs the handler for the current mode,
     * then calls post-hook. Override the run or compute/apply methods for custom control.
     */
    @Override
    public void periodic() {
        onPrePeriodic();
        switch (mode) {
            case IDLE:
                runIdle();
                break;
            case MANUAL:
                runManual();
                break;
            case POSITION:
                runPosition();
                break;
            case HOLDING:
                runHolding();
                break;
        }
        onPostPeriodic();
    }

    /** Default idle behavior: ensure zero output. Override to customize idle handling. */
    protected void runIdle() { applyPercentOutput(computeOutputPercent(Mode.IDLE, targetPositionMeters, getCurrentPosition())); }

    /** Default manual behavior: apply requested percent output. */
    protected void runManual() { applyPercentOutput(computeOutputPercent(Mode.MANUAL, targetPositionMeters, getCurrentPosition())); }

    /** Default position behavior: compute output via single overridable method, then transition to HOLDING when within tolerance. */
    protected void runPosition() {
        double output = computeOutputPercent(Mode.POSITION, targetPositionMeters, getCurrentPosition());
        applyPercentOutput(output);
        if (atTarget()) {
            setMode(Mode.HOLDING);
            holdPid.reset();
            onTargetReached(targetPositionMeters);
        }
    }

    /** Default holding behavior: compute output via single overridable method. */
    protected void runHolding() {
        double output = computeOutputPercent(Mode.HOLDING, targetPositionMeters, getCurrentPosition());
        applyPercentOutput(output);
    }

    /**
     * Single overridable output computation: includes PID, feedforward, augmentors, and clamping.
     * Override this one method if you want fully custom control for any mode.
     * @param mode current control mode
     * @param targetMeters target position (m)
     * @param currentMeters measured position (m)
     * @return motor percent output in configured limits
     */
    protected double computeOutputPercent(Mode mode, double targetMeters, double currentMeters) {
        // Pre-compute hook (can read sensors, prime state, etc.)
        onPreComputeOutput(mode, targetMeters, currentMeters);
        double basePercent;
        switch (mode) {
            case IDLE:
                basePercent = 0.0;
                break;
            case MANUAL:
                basePercent = manualPercentOutput;
                break;
            case POSITION: {
                double ffVolts = feedforward.calculate(desiredVelocity, desiredAcceleration);
                double ffPercent = nominalVoltage != 0.0 ? ffVolts / nominalVoltage : 0.0;
                double pidPercent = positionPid.calculate(currentMeters, targetMeters);
                basePercent = ffPercent + pidPercent;
                break;
            }
            case HOLDING: {
                double ffVolts = feedforward.calculate(0.0, 0.0);
                double ffPercent = nominalVoltage != 0.0 ? ffVolts / nominalVoltage : 0.0;
                double pidPercent = holdPid.calculate(currentMeters, targetMeters);
                basePercent = ffPercent + pidPercent;
                break;
            }
            default:
                basePercent = 0.0;
        }
        // Allow subclasses to adjust after base computation (without replacing it).
        double afterBase = afterBaseCompute(mode, targetMeters, currentMeters, basePercent);
        // Apply augmentors inside the single computation path
        double withAugment = applyAugmentors(afterBase, targetMeters, currentMeters);
        // Allow subclasses to adjust after augmentors
        double afterAugment = afterAugmentCompute(mode, targetMeters, currentMeters, withAugment);
        // Apply output filters (e.g., beam breaks) supplied at runtime
        double filtered = applyOutputFilters(targetMeters, currentMeters, afterAugment);
        // Post-compute hook
        onPostComputeOutput(mode, targetMeters, currentMeters, filtered);
        return clampPercent(filtered);
    }

    /**
     * Backwards-compatible wrappers that delegate to {@link #computeOutputPercent(Mode, double, double)}.
     * Prefer overriding {@code computeOutputPercent}.
     */
    @Deprecated
    protected double computePositionOutput(double targetMeters, double currentMeters) {
        return computeOutputPercent(Mode.POSITION, targetMeters, currentMeters);
    }
    @Deprecated
    protected double computeHoldOutput(double targetMeters, double currentMeters) {
        return computeOutputPercent(Mode.HOLDING, targetMeters, currentMeters);
    }

    /**
     * Send percent output to the leader motor via {@link MotorAdapter}, if present.
     * Override this to use vendor-specific control modes (e.g., MotionMagic, Spark PID).
     */
    protected void applyPercentOutput(double percent) {
        onPreApplyOutput(percent);
        
        leaderMotor.set(clampPercent(percent));
        onPostApplyOutput(percent);
    }

    // --- Output limits (soft-code clamp range) ---
    private double minOutputPercent = -1.0;
    private double maxOutputPercent = 1.0;
    /** Configure output clamp range (inclusive). */
    public void setOutputLimits(double minPercent, double maxPercent) {
        this.minOutputPercent = Math.min(minPercent, maxPercent);
        this.maxOutputPercent = Math.max(minPercent, maxPercent);
    }
    /** Current minimum output clamp. */
    public double getMinOutputPercent() { return minOutputPercent; }
    /** Current maximum output clamp. */
    public double getMaxOutputPercent() { return maxOutputPercent; }
    /** Helper to clamp to configured output range. */
    protected double clampPercent(double percent) {
        return DoubleUtils.clamp(percent, minOutputPercent, maxOutputPercent);
    }

    // Augmentor application helper
    protected double applyAugmentors(double basePercent, double targetMeters, double currentMeters) {
        double augmented = basePercent;
        for (var fn : outputAugmentors) {
            try { augmented += fn.apply(targetMeters, currentMeters); } catch (Exception ignored) {}
        }
        return augmented; 
    }

    // Public tuning and configuration methods
    public void setPositionPID(double kP, double kI, double kD) { positionPid.setPID(kP, kI, kD); }
    public void setHoldPID(double kP, double kI, double kD) { holdPid.setPID(kP, kI, kD); }
    public void setFeedforwardGains(double kS, double kG, double kV, double kA) { this.feedforward = new ElevatorFeedforward(kS, kG, kV, kA); }
    public void setDesiredMotion(double velocityMetersPerSec, double accelMetersPerSecSq) { this.desiredVelocity = velocityMetersPerSec; this.desiredAcceleration = accelMetersPerSecSq; }
    public void setNominalVoltage(double volts) { this.nominalVoltage = volts; }
    public void addOutputAugmentor(java.util.function.BiFunction<Double, Double, Double> augmentor) { if (augmentor != null) outputAugmentors.add(augmentor); }
    public boolean removeOutputAugmentor(java.util.function.BiFunction<Double, Double, Double> augmentor) { return outputAugmentors.remove(augmentor); }
    public void clearOutputAugmentors() { outputAugmentors.clear(); }

    // Lifecycle hooks

    /** Called before running the mode handler in {@link #periodic()}. */
    @Override
    protected void onPrePeriodic() {}
    /** Called after running the mode handler in {@link #periodic()}. */
    @Override
    protected void onPostPeriodic() {}
    /** Called before computing output in {@link #computeOutputPercent}. */
    protected void onPreComputeOutput(Mode mode, double targetMeters, double currentMeters) {}
    /** Called after computing and filtering output in {@link #computeOutputPercent}. */
    protected void onPostComputeOutput(Mode mode, double targetMeters, double currentMeters, double outputPercent) {}
    /** Hook to adjust output immediately after base (PID+FF) calculation. */
    protected double afterBaseCompute(Mode mode, double targetMeters, double currentMeters, double basePercent) { return basePercent; }
    /** Hook to adjust output after augmentors composition but before filters/clamp. */
    protected double afterAugmentCompute(Mode mode, double targetMeters, double currentMeters, double percent) { return percent; }
    /** Called right before sending percent to the motor controller. */
    protected void onPreApplyOutput(double percent) {}
    /** Called right after sending percent to the motor controller. */
    protected void onPostApplyOutput(double percent) {}
    /** Called exactly once when the POSITION target is declared reached. */
    protected void onTargetReached(double targetMeters) {}
    /** Called after {@link #stop()} changes the mode to IDLE. */
    @Override
    protected void onStop() {}
    /** Called whenever the mode changes. */
    protected void onModeChanged(Mode from, Mode to) {}

    /** helper to centralize mode changes and fire {@link #onModeChanged(Mode, Mode)}. */
    private void setMode(Mode newMode) {
        if (this.mode != newMode) {
            Mode prev = this.mode;
            this.mode = newMode;
            onModeChanged(prev, newMode);
        }
    }

    @Override
    public Sendable log() { return null; }

    // Configuration

    /**
     * All units are meters, meters/sec, or meters/sec^2 unless noted.
     */
    public static class ElevatorConfig {
        /** Bottom soft limit (m). */
        public final double minHeightMeters;
        /** Top soft limit (m). */
        public final double maxHeightMeters;
        /** Sensor rotations per mechanism rotation. */
        public final double gearRatio;
        /** Drum diameter (m). */
        public final double drumDiameterMeters;
        /** Nominal max velocity for planning/telemetry (m/s). */
        public final double maxVelocityMetersPerSec;
        /** Nominal max acceleration for planning/telemetry (m/s^2). */
        public final double maxAccelerationMetersPerSecSq;
        /** Position tolerance (m). */
        public final double toleranceMeters;

        private ElevatorConfig(Builder b) {
            this.minHeightMeters = b.minHeightMeters;
            this.maxHeightMeters = b.maxHeightMeters;
            this.gearRatio = b.gearRatio;
            this.drumDiameterMeters = b.drumDiameterMeters;
            this.maxVelocityMetersPerSec = b.maxVelocityMetersPerSec;
            this.maxAccelerationMetersPerSecSq = b.maxAccelerationMetersPerSecSq;
            this.toleranceMeters = b.toleranceMeters;
        }

        public static Builder builder() { return new Builder(); }

        /** Fluent builder for {@link ElevatorConfig}. */
        public static class Builder {
            private double minHeightMeters;
            private double maxHeightMeters;
            private double gearRatio;
            private double drumDiameterMeters;
            private double maxVelocityMetersPerSec;
            private double maxAccelerationMetersPerSecSq;
            private double toleranceMeters = 0.01;

            /** Bottom soft limit (m). */
            public Builder minHeightMeters(double v) { this.minHeightMeters = v; return this; }
            /** Top soft limit (m). */
            public Builder maxHeightMeters(double v) { this.maxHeightMeters = v; return this; }
            /** Sensor rotations per mechanism rotation. */
            public Builder gearRatio(double v) { this.gearRatio = v; return this; }
            /** Drum diameter (m). */
            public Builder drumDiameterMeters(double v) { this.drumDiameterMeters = v; return this; }
            /** Nominal max velocity for planning/telemetry (m/s). */
            public Builder maxVelocityMetersPerSec(double v) { this.maxVelocityMetersPerSec = v; return this; }
            /** Nominal max acceleration for planning/telemetry (m/s^2). */
            public Builder maxAccelerationMetersPerSecSq(double v) { this.maxAccelerationMetersPerSecSq = v; return this; }
            /** Position tolerance (m). */
            public Builder toleranceMeters(double v) { this.toleranceMeters = v; return this; }

            public ElevatorConfig build() { return new ElevatorConfig(this); }
        }
    }

    // ---- Output filters (e.g., beam-breaks, soft interlocks) ----
    /** Contract for modifying the computed output based on additional context (e.g., sensors). */
    @FunctionalInterface
    public interface OutputFilter { double filter(double targetMeters, double currentMeters, double proposedPercent); }
    private final java.util.List<OutputFilter> outputFilters = new java.util.ArrayList<>();
    /** Add an output filter that can modify or gate the computed output before clamping and apply. */
    public void addOutputFilter(OutputFilter filter) { if (filter != null) outputFilters.add(filter); }
    /** Remove a previously added filter. */
    public boolean removeOutputFilter(OutputFilter filter) { return outputFilters.remove(filter); }
    /** Remove all filters. */
    public void clearOutputFilters() { outputFilters.clear(); }
    /** Apply all registered filters in order. */

    protected double applyOutputFilters(double targetMeters, double currentMeters, double proposedPercent) {
        double p = proposedPercent;
        for (OutputFilter f : outputFilters) {
            try { p = f.filter(targetMeters, currentMeters, p); } catch (Exception ignored) {}
        }
        return p;
    }
}
