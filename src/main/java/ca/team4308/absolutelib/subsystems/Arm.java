package ca.team4308.absolutelib.subsystems;

import java.util.ArrayList;
import java.util.List;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.subsystems.simulation.ArmSimulation;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
// Added for IK
import edu.wpi.first.math.geometry.Translation2d;

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
     * Hook: one-time initialization.
     */
    public static class Joint {

        public enum Mode {
            IDLE, MANUAL, POSITION, HOLDING
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

        public void setOutputLimits(double minPercent, double maxPercent) {
            this.minOutputPercent = Math.min(minPercent, maxPercent);
            this.maxOutputPercent = Math.max(minPercent, maxPercent);
        }

        private double clampPercent(double p) {
            return DoubleUtils.clamp(p, minOutputPercent, maxOutputPercent);
        }

        public void setManualPercent(double pct) {
            manualPercent = clampPercent(pct);
            setMode(Mode.MANUAL);
        }

        public void setTargetAngleRadians(double radians) {
            targetRadians = DoubleUtils.clamp(radians, config.minAngleRad, config.maxAngleRad);
            setMode(Mode.POSITION);
            positionPid.reset();
        }

        public void stop() {
            manualPercent = 0;
            applyPercentOutput(0);
            setMode(Mode.IDLE);
        }
        // Optimized: return cached angle updated in periodic

        public double getAngleRadians() {
            return cachedAngleRad;
        }

        private void updateCachedAngle() {
            cachedAngleRad = encoder.getPositionMeters() * config.metersToRadians;
        }

        public double getTargetAngleRadians() {
            return targetRadians;
        }

        public boolean atTarget() {
            return Math.abs(cachedAngleRad - targetRadians) <= config.toleranceRad;
        }

        public Mode getMode() {
            return mode;
        }

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
         */
        /**
         * Single overridable computation for arm output including PID, FF,
         * augmentors, and clamping.
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

        @Deprecated
        protected double computePositionOutput(double target, double current) {
            return computeOutputPercent(Mode.POSITION, target, current);
        }

        @Deprecated
        protected double computeHoldOutput(double target, double current) {
            return computeOutputPercent(Mode.HOLDING, target, current);
        }

        public double lastAppliedVoltage = 0.0;

        protected void applyPercentOutput(double percent) {
            onPreApplyOutput(percent);
            if (!config.useSmartMotion) {
                motor.set(percent);
            }
            lastAppliedVoltage = percent * nominalVoltage;
            onPostApplyOutput(percent);
        }

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

        public void setPositionPID(double kP, double kI, double kD) {
            positionPid.setPID(kP, kI, kD);
        }

        public void setHoldPID(double kP, double kI, double kD) {
            holdPid.setPID(kP, kI, kD);
        }

        public void setFeedforwardGains(double kS, double kCos, double kV, double kA) {
            feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        }

        public void setDesiredVelocity(double velocityRadPerSec) {
            desiredVelocityRadPerSec = velocityRadPerSec;
        }

        public void setNominalVoltage(double volts) {
            nominalVoltage = volts;
        }

        public void addOutputAugmentor(java.util.function.BiFunction<Double, Double, Double> augment) {
            if (augment != null) {
                augmentors.add(augment);

            }
        }

        public boolean removeOutputAugmentor(java.util.function.BiFunction<Double, Double, Double> augment) {
            return augmentors.remove(augment);
        }

        public void clearOutputAugmentors() {
            augmentors.clear();
        }

        protected void onPrePeriodic() {
        }

        protected void onPostPeriodic() {
        }

        protected void onPreComputeOutput(Mode mode, double targetRad, double currentRad) {
        }

        protected void onPostComputeOutput(Mode mode, double targetRad, double currentRad, double outputPercent) {
        }

        protected double afterBaseCompute(Mode mode, double targetRad, double currentRad, double basePercent) {
            return basePercent;
        }

        protected double afterAugmentCompute(Mode mode, double targetRad, double currentRad, double percent) {
            return percent;
        }

        protected void onPreApplyOutput(double percent) {
        }

        protected void onPostApplyOutput(double percent) {
        }

        protected void onTargetReached(double targetRad) {
        }

        protected void onModeChanged(Mode from, Mode to) {
        }

        private void setMode(Mode newMode) {
            if (mode != newMode) {
                Mode prev = mode;
                mode = newMode;
                onModeChanged(prev, newMode);
            }
        }

        @FunctionalInterface
        public interface OutputFilter {

            double filter(double targetRad, double currentRad, double proposedPercent);
        }
        private final java.util.List<OutputFilter> outputFilters = new java.util.ArrayList<>();

        public void addOutputFilter(OutputFilter f) {
            if (f != null) {
                outputFilters.add(f);

            }
        }

        public boolean removeOutputFilter(OutputFilter f) {
            return outputFilters.remove(f);
        }

        public void clearOutputFilters() {
            outputFilters.clear();
        }

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
         */
        public MotorWrapper getMotor() {
            return motor;
        }

        /**
         * Get the encoder for simulation access.
         */
        public EncoderWrapper getEncoder() {
            return encoder;
        }
    }

    public static class JointConfig {

        public final double minAngleRad;
        public final double maxAngleRad;
        public final double toleranceRad;
        public final double metersToRadians;
        // Added for IK
        public final double linkLengthMeters;

        public boolean useSmartMotion = false;

        private JointConfig(Builder b) {
            minAngleRad = b.minAngleRad;
            maxAngleRad = b.maxAngleRad;
            toleranceRad = b.toleranceRad;
            metersToRadians = b.metersToRadians;
            linkLengthMeters = b.linkLengthMeters;
            useSmartMotion = b.useSmartMotion;
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {

            private double minAngleRad = 0.0, maxAngleRad = Math.PI, toleranceRad = Math.toRadians(2), metersToRadians = 1.0;
            private double linkLengthMeters = 1.0;
            private boolean useSmartMotion = false;

            public Builder minAngleRad(double v) {
                minAngleRad = v;
                return this;
            }

            public Builder maxAngleRad(double v) {
                maxAngleRad = v;
                return this;
            }

            public Builder toleranceRad(double v) {
                toleranceRad = v;
                return this;
            }

            public Builder metersToRadians(double v) {
                metersToRadians = v;
                return this;
            }

            public Builder linkLengthMeters(double v) {
                linkLengthMeters = v;
                return this;
            }

            public Builder useSmartMotion(boolean v) {
                useSmartMotion = v;
                return this;
            }

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

    public Arm() {
    }

    public Joint addJoint(MotorWrapper motor, MotorConfig config, EncoderWrapper encoder, JointConfig jointConfig) {
        double initialAngle = encoder.getPositionMeters() * jointConfig.metersToRadians;
        Joint j = new Joint(motor, config, encoder, jointConfig, initialAngle);
        joints.add(j);
        cachedIKAngles = new double[joints.size()]; // resize cache

        if (jointConfig.useSmartMotion) {

        }

        return j;
    }

    public List<Joint> getJoints() {
        return joints;
    }

    // Direct angle control (non-IK)
    public void setTargetAngles(double... radians) {
        ikMode = false;
        int n = Math.min(radians.length, joints.size());
        for (int i = 0; i < n; i++) {
            joints.get(i).setTargetAngleRadians(radians[i]);
        }
    }

    public void setManualPercents(double... percents) {
        ikMode = false;
        int n = Math.min(percents.length, joints.size());
        for (int i = 0; i < n; i++) {
            joints.get(i).setManualPercent(percents[i]);
        }
    }

    // IK control
    public void setGoalPose(double x, double y) {
        this.goalPose = new Translation2d(x, y);
        this.ikMode = true;
        solveIK();
    }

    public void setGoalPose(Translation2d pose) {
        setGoalPose(pose.getX(), pose.getY());
    }

    public boolean isIKMode() {
        return ikMode;
    }

    public boolean isIKSolved() {
        return ikSolved;
    }

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

    public void enableSimulation(boolean enable) {
        this.enableSimulation = enable;
    }

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
