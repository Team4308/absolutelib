package ca.team4308.absolutelib.subsystems;

import java.util.ArrayList;
import java.util.List;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.subsystems.simulation.ArmSimulation;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;

/**
 * Multi-DOF Arm subsystem: per-joint modes (IDLE, MANUAL, POSITION, HOLDING)
 * with IK support
 */
public class Arm extends AbsoluteSubsystem {

    /**
     * Invoke once to allow subclasses to do one-time setup.
     */
    @Override
    public final void initialize() {
        onInitialize();
    }

    /**
     * A single rotary joint in the arm, with PID, feedforward, and output filtering.
     */
    public static class Joint {

        /** Operating mode for a joint. */
        public enum Mode {
            /** Joint is idle and not producing output. */
            IDLE,
            /** Joint is under direct manual percent control. */
            MANUAL,
            /** Joint is moving to a target angle via PID. */
            POSITION,
            /** Joint is holding its current angle. */
            HOLDING
        }
        private final MotorWrapper motor;
        private EncoderWrapper encoder;
        private final JointConfig config;
        private Mode mode = Mode.IDLE;
        private double targetRadians = 0.0;
        private double manualPercent = 0.0;
        private final PIDController positionPid = new PIDController(0, 0, 0);
        private final PIDController holdPid = new PIDController(0, 0, 0);
        private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
        private double desiredVelocityRadPerSec = 0.0;
        private double nominalVoltage = 12.0;
        private final java.util.List<java.util.function.BiFunction<Double, Double, Double>> augmentors = new java.util.ArrayList<>();
        // Cached angle for performance
        private double cachedAngleRad = 0.0;

        /**
         * Creates a new joint.
         *
         * @param motor           motor driving this joint
         * @param motorConfig     motor configuration to apply (may be null)
         * @param encoder         encoder measuring joint angle
         * @param config          joint configuration (limits, tolerance, etc.)
         * @param initialAngleRad initial angle in radians
         */
        public Joint(MotorWrapper motor, MotorConfig motorConfig, EncoderWrapper encoder, JointConfig config, double initialAngleRad) {
            this.motor = motor;
            this.encoder = encoder;
            this.config = config;
            if (motorConfig != null) {
                motor.applyMotorConfig(motorConfig);
            }

            this.targetRadians = DoubleUtils.clamp(initialAngleRad, config.minAngleRad, config.maxAngleRad);
            this.mode = Mode.HOLDING;
            this.cachedAngleRad = initialAngleRad;
        }
        private double minOutputPercent = -1.0;
        private double maxOutputPercent = 1.0;

        /**
         * Sets the min and max output percent limits for this joint.
         *
         * @param minPercent minimum output percent
         * @param maxPercent maximum output percent
         */
        public void setOutputLimits(double minPercent, double maxPercent) {
            this.minOutputPercent = Math.min(minPercent, maxPercent);
            this.maxOutputPercent = Math.max(minPercent, maxPercent);
        }

        private double clampPercent(double p) {
            return DoubleUtils.clamp(p, minOutputPercent, maxOutputPercent);
        }

        /**
         * Sets the joint to manual mode with the given output percent.
         *
         * @param pct output percent in [-1, 1]
         */
        public void setManualPercent(double pct) {
            manualPercent = clampPercent(pct);
            setMode(Mode.MANUAL);
        }

        /**
         * Sets the target angle and switches the joint to position mode.
         *
         * @param radians desired angle in radians
         */
        public void setTargetAngleRadians(double radians) {
            targetRadians = DoubleUtils.clamp(radians, config.minAngleRad, config.maxAngleRad);
            setMode(Mode.POSITION);
            positionPid.reset();
        }

        /** Stops the joint and sets it to idle mode. */
        public void stop() {
            manualPercent = 0;
            applyPercentOutput(0);
            setMode(Mode.IDLE);
        }
        // Optimized: return cached angle updated in periodic

        /**
         * Returns the current joint angle in radians.
         *
         * @return current angle in radians
         */
        public double getAngleRadians() {
            return cachedAngleRad;
        }

        private void updateCachedAngle() {
            cachedAngleRad = encoder.getPositionMeters() * config.metersToRadians;
        }

        /**
         * Returns the target joint angle in radians.
         *
         * @return target angle in radians
         */
        public double getTargetAngleRadians() {
            return targetRadians;
        }

        /**
         * Returns true if the joint is within tolerance of its target.
         *
         * @return true if at target
         */
        public boolean atTarget() {
            return Math.abs(cachedAngleRad - targetRadians) <= config.toleranceRad;
        }

        /**
         * Returns the current operating mode of this joint.
         *
         * @return current mode
         */
        public Mode getMode() {
            return mode;
        }

        /**
         * Replaces the encoder used by this joint.
         *
         * @param e the new encoder
         */
        public void setEncoder(EncoderWrapper e) {
            this.encoder = e;
        }

        void periodic() {
            updateCachedAngle();
            onPrePeriodic();
            double out = computeOutputPercent(mode, targetRadians, cachedAngleRad);
            applyPercentOutput(out);
            if (mode == Mode.POSITION && atTarget()) {
                setMode(Mode.HOLDING);
                holdPid.reset();
                onTargetReached(targetRadians);
            }
            onPostPeriodic();
        }

        /**
         * Single overridable computation for arm output including PID, FF,
         * augmentors, and clamping.
         *
         * @param mode    current joint operating mode
         * @param target  target angle in radians
         * @param current current angle in radians
         * @return output percent in [-1, 1]
         */
        protected double computeOutputPercent(Mode mode, double target, double current) {
            onPreComputeOutput(mode, target, current);
            double base;
            switch (mode) {
                case IDLE:
                    base = 0.0;
                    break;
                case MANUAL:
                    base = manualPercent;
                    break;
                case POSITION: {
                    if (config.useSmartMotion) {
                        double ffVolts = feedforward.calculate(current, desiredVelocityRadPerSec);
                        double targetNative = target / config.metersToRadians;

                        motor.setSmartPosition(targetNative, ffVolts);
                        base = 0; // Output handled by motor
                    } else {
                        double ffVolts = feedforward.calculate(current, desiredVelocityRadPerSec);
                        double ffPercent = nominalVoltage != 0 ? ffVolts / nominalVoltage : 0.0;
                        double pidPercent = positionPid.calculate(current, target);
                        base = ffPercent + pidPercent;
                    }
                    break;
                }
                case HOLDING: {
                    if (config.useSmartMotion) {
                        double ffVolts = feedforward.calculate(current, 0.0);
                        double targetNative = target / config.metersToRadians;
                        motor.setSmartPosition(targetNative, ffVolts);
                        base = 0;
                    } else {
                        double ffVolts = feedforward.calculate(current, 0.0);
                        double ffPercent = nominalVoltage != 0 ? ffVolts / nominalVoltage : 0.0;
                        double pidPercent = holdPid.calculate(current, target);
                        base = ffPercent + pidPercent;
                    }
                    break;
                }
                default:
                    base = 0.0;
            }
            double afterBase = afterBaseCompute(mode, target, current, base);
            double withAug = applyAugmentors(afterBase, target, current);
            double afterAug = afterAugmentCompute(mode, target, current, withAug);
            double filtered = applyOutputFilters(target, current, afterAug);
            onPostComputeOutput(mode, target, current, filtered);
            return clampPercent(filtered);
        }

        /**
         * @deprecated Use {@link #computeOutputPercent(Mode, double, double)} instead.
         * @param target  target angle in radians
         * @param current current angle in radians
         * @return output percent
         */
        @Deprecated
        protected double computePositionOutput(double target, double current) {
            return computeOutputPercent(Mode.POSITION, target, current);
        }

        /**
         * @deprecated Use {@link #computeOutputPercent(Mode, double, double)} instead.
         * @param target  target angle in radians
         * @param current current angle in radians
         * @return output percent
         */
        @Deprecated
        protected double computeHoldOutput(double target, double current) {
            return computeOutputPercent(Mode.HOLDING, target, current);
        }

        /** Last voltage applied to the motor (for simulation). */
        public double lastAppliedVoltage = 0.0;

        /**
         * Sends the computed percent output to the motor.
         *
         * @param percent the output percent to apply
         */
        protected void applyPercentOutput(double percent) {
            onPreApplyOutput(percent);
            if (!config.useSmartMotion) {
                motor.set(percent);
            }
            lastAppliedVoltage = percent * nominalVoltage;
            onPostApplyOutput(percent);
        }

        /**
         * Applies all registered output augmentors to the base output.
         *
         * @param base    base output percent
         * @param target  target angle in radians
         * @param current current angle in radians
         * @return augmented output percent
         */
        protected double applyAugmentors(double base, double target, double current) {
            double out = base;
            for (var fn : augmentors) {
                try {
                    out += fn.apply(target, current);
                } catch (Exception ignored) {
                }
            }
            return out;
        }

        /**
         * Sets the position PID gains.
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        public void setPositionPID(double kP, double kI, double kD) {
            positionPid.setPID(kP, kI, kD);
        }

        /**
         * Sets the hold PID gains.
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        public void setHoldPID(double kP, double kI, double kD) {
            holdPid.setPID(kP, kI, kD);
        }

        /**
         * Sets the arm feedforward gains.
         *
         * @param kS   static gain
         * @param kCos cosine (gravity) gain
         * @param kV   velocity gain
         * @param kA   acceleration gain
         */
        public void setFeedforwardGains(double kS, double kCos, double kV, double kA) {
            feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        }

        /**
         * Sets the desired joint velocity in radians per second for feedforward.
         *
         * @param velocityRadPerSec desired velocity in rad/s
         */
        public void setDesiredVelocity(double velocityRadPerSec) {
            desiredVelocityRadPerSec = velocityRadPerSec;
        }

        /**
         * Sets the nominal battery voltage used for percent-to-voltage conversion.
         *
         * @param volts nominal voltage
         */
        public void setNominalVoltage(double volts) {
            nominalVoltage = volts;
        }

        /**
         * Adds an output augmentor that contributes additional output based on target and current angles.
         *
         * @param augment the augmentor function
         */
        public void addOutputAugmentor(java.util.function.BiFunction<Double, Double, Double> augment) {
            if (augment != null) {
                augmentors.add(augment);

            }
        }

        /**
         * Removes a previously registered output augmentor.
         *
         * @param augment the augmentor to remove
         * @return true if the augmentor was found and removed
         */
        public boolean removeOutputAugmentor(java.util.function.BiFunction<Double, Double, Double> augment) {
            return augmentors.remove(augment);
        }

        /** Removes all registered output augmentors. */
        public void clearOutputAugmentors() {
            augmentors.clear();
        }

        /** Hook called before the periodic computation. */
        protected void onPrePeriodic() {
        }

        /** Hook called after the periodic computation. */
        protected void onPostPeriodic() {
        }

        /**
         * Hook called before computing the output.
         *
         * @param mode       current mode
         * @param targetRad  target angle in radians
         * @param currentRad current angle in radians
         */
        protected void onPreComputeOutput(Mode mode, double targetRad, double currentRad) {
        }

        /**
         * Hook called after computing the output.
         *
         * @param mode          current mode
         * @param targetRad     target angle in radians
         * @param currentRad    current angle in radians
         * @param outputPercent computed output percent
         */
        protected void onPostComputeOutput(Mode mode, double targetRad, double currentRad, double outputPercent) {
        }

        /**
         * Hook called after the base output is computed, before augmentors.
         *
         * @param mode        current mode
         * @param targetRad   target angle in radians
         * @param currentRad  current angle in radians
         * @param basePercent base output percent
         * @return modified output percent
         */
        protected double afterBaseCompute(Mode mode, double targetRad, double currentRad, double basePercent) {
            return basePercent;
        }

        /**
         * Hook called after augmentors are applied.
         *
         * @param mode       current mode
         * @param targetRad  target angle in radians
         * @param currentRad current angle in radians
         * @param percent    output percent after augmentors
         * @return modified output percent
         */
        protected double afterAugmentCompute(Mode mode, double targetRad, double currentRad, double percent) {
            return percent;
        }

        /**
         * Hook called before applying the output to the motor.
         *
         * @param percent the output percent about to be applied
         */
        protected void onPreApplyOutput(double percent) {
        }

        /**
         * Hook called after applying the output to the motor.
         *
         * @param percent the output percent that was applied
         */
        protected void onPostApplyOutput(double percent) {
        }

        /**
         * Hook called when the joint reaches its target angle.
         *
         * @param targetRad the target angle that was reached
         */
        protected void onTargetReached(double targetRad) {
        }

        /**
         * Hook called when the joint mode changes.
         *
         * @param from previous mode
         * @param to   new mode
         */
        protected void onModeChanged(Mode from, Mode to) {
        }

        private void setMode(Mode newMode) {
            if (mode != newMode) {
                Mode prev = mode;
                mode = newMode;
                onModeChanged(prev, newMode);
            }
        }

        /** Functional interface for filtering joint output. */
        @FunctionalInterface
        public interface OutputFilter {

            /**
             * Filters the motor output for a joint.
             *
             * @param targetRad       target angle in radians
             * @param currentRad      current angle in radians
             * @param proposedPercent the proposed output percent
             * @return the filtered output percent
             */
            double filter(double targetRad, double currentRad, double proposedPercent);
        }
        private final java.util.List<OutputFilter> outputFilters = new java.util.ArrayList<>();

        /**
         * Adds an output filter to the processing chain.
         *
         * @param f the output filter to add
         */
        public void addOutputFilter(OutputFilter f) {
            if (f != null) {
                outputFilters.add(f);

            }
        }

        /**
         * Removes a previously registered output filter.
         *
         * @param f the filter to remove
         * @return true if the filter was found and removed
         */
        public boolean removeOutputFilter(OutputFilter f) {
            return outputFilters.remove(f);
        }

        /** Removes all registered output filters. */
        public void clearOutputFilters() {
            outputFilters.clear();
        }

        /**
         * Runs the output through all registered filters.
         *
         * @param targetRad       target angle in radians
         * @param currentRad      current angle in radians
         * @param proposedPercent the output percent before filtering
         * @return filtered output percent
         */
        protected double applyOutputFilters(double targetRad, double currentRad, double proposedPercent) {
            double p = proposedPercent;
            for (OutputFilter f : outputFilters) {
                try {
                    p = f.filter(targetRad, currentRad, p);
                } catch (Exception ignored) {
                }
            }
            return p;
        }

        /**
         * Get the motor for simulation access.
         *
         * @return the motor wrapper
         */
        public MotorWrapper getMotor() {
            return motor;
        }

        /**
         * Get the encoder for simulation access.
         *
         * @return the encoder wrapper
         */
        public EncoderWrapper getEncoder() {
            return encoder;
        }
    }

    /** Immutable configuration for an arm joint. */
    public static class JointConfig {

        /** Minimum allowed angle in radians. */
        public final double minAngleRad;
        /** Maximum allowed angle in radians. */
        public final double maxAngleRad;
        /** Tolerance for at-target checks in radians. */
        public final double toleranceRad;
        /** Conversion factor from meters to radians. */
        public final double metersToRadians;
        /** Link length in meters for IK calculations. */
        public final double linkLengthMeters;

        /** Whether to use smart motion profiling. */
        public boolean useSmartMotion = false;

        private JointConfig(Builder b) {
            minAngleRad = b.minAngleRad;
            maxAngleRad = b.maxAngleRad;
            toleranceRad = b.toleranceRad;
            metersToRadians = b.metersToRadians;
            linkLengthMeters = b.linkLengthMeters;
            useSmartMotion = b.useSmartMotion;
        }

        /**
         * Creates a new builder for {@link JointConfig}.
         *
         * @return a new builder instance
         */
        public static Builder builder() {
            return new Builder();
        }

        /** Builder for {@link JointConfig}. */
        public static class Builder {

            private double minAngleRad = 0.0, maxAngleRad = Math.PI, toleranceRad = Math.toRadians(2), metersToRadians = 1.0;
            private double linkLengthMeters = 1.0;
            private boolean useSmartMotion = false;

            /**
             * Sets the minimum angle.
             *
             * @param v minimum angle in radians
             * @return this builder
             */
            public Builder minAngleRad(double v) {
                minAngleRad = v;
                return this;
            }

            /**
             * Sets the maximum angle.
             *
             * @param v maximum angle in radians
             * @return this builder
             */
            public Builder maxAngleRad(double v) {
                maxAngleRad = v;
                return this;
            }

            /**
             * Sets the at-target tolerance.
             *
             * @param v tolerance in radians
             * @return this builder
             */
            public Builder toleranceRad(double v) {
                toleranceRad = v;
                return this;
            }

            /**
             * Sets the meters-to-radians conversion factor.
             *
             * @param v conversion factor
             * @return this builder
             */
            public Builder metersToRadians(double v) {
                metersToRadians = v;
                return this;
            }

            /**
             * Sets the link length for IK.
             *
             * @param v link length in meters
             * @return this builder
             */
            public Builder linkLengthMeters(double v) {
                linkLengthMeters = v;
                return this;
            }

            /**
             * Enables or disables smart motion profiling.
             *
             * @param v true to enable smart motion
             * @return this builder
             */
            public Builder useSmartMotion(boolean v) {
                useSmartMotion = v;
                return this;
            }

            /**
             * Builds an immutable {@link JointConfig}.
             *
             * @return the joint configuration
             */
            public JointConfig build() {
                return new JointConfig(this);
            }
        }
    }

    private final List<Joint> joints = new ArrayList<>();
    // IK state
    private boolean ikMode = false;
    private Translation2d goalPose = new Translation2d();
    private double[] cachedIKAngles = null;
    private boolean ikSolved = false;
    // IK tuning
    private int ikMaxIterations = 40;
    private double ikPosToleranceMeters = 0.01;

    /** Creates a new arm with no joints. */
    public Arm() {
    }

    /**
     * Adds a joint to this arm.
     *
     * @param motor       motor driving the joint
     * @param config      motor configuration
     * @param encoder     encoder for the joint
     * @param jointConfig joint configuration
     * @return the created joint
     */
    public Joint addJoint(MotorWrapper motor, MotorConfig config, EncoderWrapper encoder, JointConfig jointConfig) {
        double initialAngle = encoder.getPositionMeters() * jointConfig.metersToRadians;
        Joint j = new Joint(motor, config, encoder, jointConfig, initialAngle);
        joints.add(j);
        cachedIKAngles = new double[joints.size()]; // resize cache

        if (jointConfig.useSmartMotion) {

        }

        return j;
    }

    /**
     * Returns the list of joints in this arm.
     *
     * @return unmodifiable view of joints
     */
    public List<Joint> getJoints() {
        return joints;
    }

    // Direct angle control (non-IK)
    /**
     * Sets target angles (in radians) for each joint directly, disabling IK mode.
     *
     * @param radians target angles, one per joint
     */
    public void setTargetAngles(double... radians) {
        ikMode = false;
        int n = Math.min(radians.length, joints.size());
        for (int i = 0; i < n; i++) {
            joints.get(i).setTargetAngleRadians(radians[i]);
        }
    }

    /**
     * Sets manual percent outputs for each joint, disabling IK mode.
     *
     * @param percents output percents, one per joint
     */
    public void setManualPercents(double... percents) {
        ikMode = false;
        int n = Math.min(percents.length, joints.size());
        for (int i = 0; i < n; i++) {
            joints.get(i).setManualPercent(percents[i]);
        }
    }

    // IK control
    /**
     * Sets the IK goal pose and solves for joint angles.
     *
     * @param x target X in meters
     * @param y target Y in meters
     */
    public void setGoalPose(double x, double y) {
        this.goalPose = new Translation2d(x, y);
        this.ikMode = true;
        solveIK();
    }

    /**
     * Sets the IK goal pose and solves for joint angles.
     *
     * @param pose target position
     */
    public void setGoalPose(Translation2d pose) {
        setGoalPose(pose.getX(), pose.getY());
    }

    /**
     * Returns true if the arm is currently in IK mode.
     *
     * @return true if in IK mode
     */
    public boolean isIKMode() {
        return ikMode;
    }

    /**
     * Returns true if the last IK solve converged.
     *
     * @return true if converged
     */
    public boolean isIKSolved() {
        return ikSolved;
    }

    /**
     * Returns the current IK goal pose.
     *
     * @return goal position
     */
    public Translation2d getGoalPose() {
        return goalPose;
    }

    // IK solver
    private void solveIK() {
        if (joints.size() < 2) {
            logWarn("IK requires at least 2 joints");
            ikSolved = false;
            return;
        }
        if (joints.size() == 2) {
            solveIK2Link();
            return;
        }
        // Multi-joint CCD
        double totalLength = 0.0;
        double longest = 0.0;
        for (Joint j : joints) {
            totalLength += j.config.linkLengthMeters;
            longest = Math.max(longest, j.config.linkLengthMeters);
        }
        double minReach = Math.max(0.0, longest - (totalLength - longest));
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        double distToGoal = Math.hypot(goalX, goalY);
        if (distToGoal > totalLength + 1e-6 || distToGoal < minReach - 1e-6) {
            logThrottle("ik_unreachable", 500, "Goal unreachable (dist=" + distToGoal + ")");
            ikSolved = false;
            return;
        }
        // Initialize working angle array
        for (int i = 0; i < joints.size(); i++) {
            cachedIKAngles[i] = joints.get(i).getAngleRadians();
        }
        for (int iter = 0; iter < ikMaxIterations; iter++) {
            // Current end effector
            Translation2d end = computeEndEffector(cachedIKAngles);
            if (end.getDistance(goalPose) <= ikPosToleranceMeters) {
                ikSolved = true;
                break;
            }
            // Adjust joints from last-1 down to 0
            for (int jIdx = joints.size() - 1; jIdx >= 0; jIdx--) {
                Translation2d jointPos = computeJointPosition(cachedIKAngles, jIdx);
                Translation2d currentEnd = computeEndEffector(cachedIKAngles);

                Translation2d vCur = new Translation2d(
                        currentEnd.getX() - jointPos.getX(),
                        currentEnd.getY() - jointPos.getY()
                );
                Translation2d vTar = new Translation2d(
                        goalX - jointPos.getX(),
                        goalY - jointPos.getY()
                );
                double angCur = Math.atan2(vCur.getY(), vCur.getX());
                double angTar = Math.atan2(vTar.getY(), vTar.getX());
                double delta = angTar - angCur;
                // Normalize to [-pi, pi]
                delta = Math.atan2(Math.sin(delta), Math.cos(delta));

                double newAngle = cachedIKAngles[jIdx] + delta;
                // Clamp
                Joint joint = joints.get(jIdx);
                newAngle = DoubleUtils.clamp(newAngle, joint.config.minAngleRad, joint.config.maxAngleRad);
                cachedIKAngles[jIdx] = newAngle;
            }
        }
        if (!ikSolved) {
            // Final check
            Translation2d end = computeEndEffector(cachedIKAngles);
            if (end.getDistance(goalPose) <= ikPosToleranceMeters) {
                ikSolved = true;
            }
        }
        if (ikSolved) {
            for (int i = 0; i < joints.size(); i++) {
                joints.get(i).setTargetAngleRadians(cachedIKAngles[i]);
                recordOutput("ik/joint" + i + "Deg", Math.toDegrees(cachedIKAngles[i]));
            }
            recordOutput("ik/solved", true);
        } else {
            logThrottle("ik_not_converged", 500, "IK did not converge");
            recordOutput("ik/solved", false);
        }
    }

    private void solveIK2Link() {
        double L1 = joints.get(0).config.linkLengthMeters;
        double L2 = joints.get(1).config.linkLengthMeters;
        double x = goalPose.getX();
        double y = goalPose.getY();
        double distSq = x * x + y * y;
        double dist = Math.sqrt(distSq);

        if (dist > L1 + L2 || dist < Math.abs(L1 - L2)) {
            logThrottle("ik_unreachable", 500, "Goal unreachable: dist =" + dist);
            ikSolved = false;
            return;
        }

        double cosTheta2 = (distSq - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        cosTheta2 = DoubleUtils.clamp(cosTheta2, -1.0, 1.0);
        double theta2 = Math.acos(cosTheta2);

        double k1 = L1 + L2 * Math.cos(theta2);
        double k2 = L2 * Math.sin(theta2);
        double theta1 = Math.atan2(y, x) - Math.atan2(k2, k1);

        theta1 = DoubleUtils.clamp(theta1, joints.get(0).config.minAngleRad, joints.get(0).config.maxAngleRad);
        theta2 = DoubleUtils.clamp(theta2, joints.get(1).config.minAngleRad, joints.get(1).config.maxAngleRad);

        cachedIKAngles[0] = theta1;
        cachedIKAngles[1] = theta2;

        joints.get(0).setTargetAngleRadians(theta1);
        joints.get(1).setTargetAngleRadians(theta2);

        ikSolved = true;

        recordOutput("ik/theta1Deg", Math.toDegrees(theta1));
        recordOutput("ik/theta2Deg", Math.toDegrees(theta2));
        recordOutput("ik/solved", true);
        for (int i = 2; i < joints.size(); i++) {
            // Add ltr joints at current angles
        }
    }

    /**
     * Returns true if all joints are within tolerance of their targets.
     *
     * @return true if all joints are at target
     */
    public boolean allAtTargets() {
        for (Joint j : joints) {
            if (!j.atTarget()) {
                return false;
            }
        }
        return true;
    }

    // Simulation
    private ca.team4308.absolutelib.subsystems.simulation.ArmSimulation simulation;
    private boolean enableSimulation = true;

    /**
     * Enables or disables arm simulation.
     *
     * @param enable true to enable simulation
     */
    public void enableSimulation(boolean enable) {
        this.enableSimulation = enable;
    }

    /**
     * Returns the arm simulation instance, or null if not initialized.
     *
     * @return simulation instance or null
     */
    public ca.team4308.absolutelib.subsystems.simulation.ArmSimulation getSimulation() {
        return simulation;
    }

    private void initSimulation() {
        if (simulation != null) {
            return;
        }

        ca.team4308.absolutelib.subsystems.simulation.ArmSimulation.Config simCfg = new ca.team4308.absolutelib.subsystems.simulation.ArmSimulation.Config();
        for (Joint j : joints) {
            ca.team4308.absolutelib.subsystems.simulation.ArmSimulation.JointSimConfig jCfg = new ca.team4308.absolutelib.subsystems.simulation.ArmSimulation.JointSimConfig();
            jCfg.gearbox = edu.wpi.first.math.system.plant.DCMotor.getNEO(1);
            jCfg.gearRatio = 100.0;
            jCfg.linkLengthMeters = j.config.linkLengthMeters;
            jCfg.linkMassKg = 2.0;
            jCfg.minAngleRad = j.config.minAngleRad;
            jCfg.maxAngleRad = j.config.maxAngleRad;
            jCfg.startAngleRad = j.getAngleRadians();
            simCfg.addJoint(jCfg);
        }

        simulation = new ArmSimulation("Arm", simCfg, this);
        simulation.initialize();
    }

    @Override
    protected void onInitialize() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation() && enableSimulation) {
            initSimulation();
        }
    }

    @Override
    public void periodic() {
        onPrePeriodic();
        for (Joint j : joints) {
            j.periodic();
        }

        if (simulation != null) {
            for (int i = 0; i < joints.size(); i++) {
                simulation.setJointVoltage(i, joints.get(i).lastAppliedVoltage);
            }
            simulation.periodic();
        }

        onPostPeriodic();
    }

    @Override
    public Sendable log() {
        return null;
    }

    private Translation2d computeEndEffector(double[] angles) {
        double x = 0.0, y = 0.0, accAngle = 0.0;
        for (int i = 0; i < joints.size(); i++) {
            accAngle += angles[i];
            double L = joints.get(i).config.linkLengthMeters;
            x += L * Math.cos(accAngle);
            y += L * Math.sin(accAngle);
        }
        return new Translation2d(x, y);
    }

    private Translation2d computeJointPosition(double[] angles, int jointIndex) {
        double x = 0.0, y = 0.0, accAngle = 0.0;
        for (int i = 0; i < jointIndex; i++) {
            accAngle += angles[i];
            double L = joints.get(i).config.linkLengthMeters;
            x += L * Math.cos(accAngle);
            y += L * Math.sin(accAngle);
        }
        return new Translation2d(x, y);
    }
}
