package ca.team4308.absolutelib.subsystems;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;

import ca.team4308.absolutelib.subsystems.Arm.Joint.Mode;
import ca.team4308.absolutelib.subsystems.simulation.PivotSimulation;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class Pivot extends AbsoluteSubsystem {

    public static class Config {

        // MOTORS AND ENCODER
        public MotorWrapper leader;
        public MotorWrapper[] followers = new MotorWrapper[0];
        public EncoderWrapper encoder;
        public boolean encoderInverted = false;
        public double angleOffsetDeg = 0.0;
        public boolean inverted = false;
        // MECHANICAL
        public double gearRatio = 1.0;
        public double minAngleDeg = -180.0;
        public double maxAngleDeg = 180.0;
        public double toleranceDeg = 1.0;
        public double weightKG = 5.0;
        public double lengthMeters = 0.5;

        // PID FF
        public double kP = 0.0, kI = 0.0, kD = 0.0;
        public double kS = 0.0, kG = 0.0, kV = 0.0, kA = 0.0;
        // MOTION PROFILE
        public double maxVelocityDegPerSec = 360.0;
        public double maxAccelerationDegPerSecSq = 720.0;
        // CONFIG
        public PivotSimulation.Config simulationConfig = null;
        public boolean enableSimulation = true;

        public boolean useSmartMotion = false;

        public Config withWeightKG(double kg) {
            weightKG = kg;
            return this;
        }

        public Config withLengthMeters(double meters) {
            lengthMeters = meters;
            return this;
        }

        public Config withLeader(MotorWrapper m) {
            this.leader = m;
            return this;
        }

        public Config withFollowers(MotorWrapper... m) {
            this.followers = m;
            return this;
        }

        public Config withEncoder(EncoderWrapper e) {
            this.encoder = e;
            return this;
        }

        public Config encoderInverted(boolean inv) {
            encoderInverted = inv;
            return this;
        }

        public Config angleOffsetDeg(double offset) {
            angleOffsetDeg = offset;
            return this;
        }

        public Config pid(double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
            return this;
        }

        public Config ff(double s, double g, double v, double a) {
            kS = s;
            kG = g;
            kV = v;
            kA = a;
            return this;
        }

        public Config motion(double maxVelDeg, double maxAccelDeg) {
            this.maxVelocityDegPerSec = maxVelDeg;
            this.maxAccelerationDegPerSecSq = maxAccelDeg;
            return this;
        }

        public Config gear(double ratio) {
            gearRatio = ratio;
            return this;
        }

        public Config limits(double minDeg, double maxDeg) {
            minAngleDeg = minDeg;
            maxAngleDeg = maxDeg;
            return this;
        }

        public Config tolerance(double tolDeg) {
            toleranceDeg = tolDeg;
            return this;
        }

        public Config inverted(boolean inv) {
            inverted = inv;
            return this;
        }

        public Config withSimulation(PivotSimulation.Config simCfg) {
            this.simulationConfig = simCfg;
            return this;
        }

        public Config enableSimulation(boolean enable) {
            this.enableSimulation = enable;
            return this;
        }

        public Config useSmartMotion(boolean enable) {
            this.useSmartMotion = enable;
            return this;
        }
    }

    private final MotorWrapper leader;
    private final List<MotorWrapper> followers = new ArrayList<>();

    private final ProfiledPIDController pid;

    private ArmFeedforward ff;
    private final Config cfg;

    private double targetAngleRad = 0.0;
    private boolean enabled = false;
    private boolean manualMode = false;
    private double manualVoltage = 0.0;

    private double lastAppliedVoltage = 0.0;

    private final boolean encoderIsAbsolute;
    private double absoluteEncoderInitialOffset = 0.0;

    // Simulation
    private PivotSimulation simulation;

    public Pivot(Config config) {
        this.cfg = config;
        leader = cfg.leader;
        for (MotorWrapper f : cfg.followers) {
            f.follow(leader);
            followers.add(f);
        }

        pid = new ProfiledPIDController(
                cfg.kP, cfg.kI, cfg.kD,
                new TrapezoidProfile.Constraints(
                        Math.toRadians(cfg.maxVelocityDegPerSec),
                        Math.toRadians(cfg.maxAccelerationDegPerSecSq)));

        pid.setTolerance(Math.toRadians(cfg.toleranceDeg));
        ff = new ArmFeedforward(cfg.kS, cfg.kG, cfg.kV, cfg.kA);

        encoderIsAbsolute = (cfg.encoder != null) && cfg.encoder.isAbsolute();
        if (encoderIsAbsolute && cfg.encoder != null) {
            absoluteEncoderInitialOffset = cfg.encoder.getPositionMeters();
        }

        if (cfg.useSmartMotion) {

            double cruiseRotPerSec = cfg.maxVelocityDegPerSec / 360.0 * cfg.gearRatio;
            double accelRotPerSecSq = cfg.maxAccelerationDegPerSecSq / 360.0 * cfg.gearRatio;

            // Apply PID to motor
            MotorWrapper.UnifiedMotorConfig motorCfg = MotorWrapper.UnifiedMotorConfig.builder()
                    .kP(cfg.kP)
                    .kI(cfg.kI)
                    .kD(cfg.kD)
                    .kF(cfg.kV) // Using kV as kF for now, though often kF is 0 for position control
                    .motionCruiseVelocity(cruiseRotPerSec)
                    .motionAcceleration(accelRotPerSecSq);

            leader.applyMotorConfig(motorCfg);
        }
    }

    public Pivot() {
        throw new IllegalStateException("Use Pivot(new Config().withLeader(...).withEncoder(...))");
    }

    protected void onInitialize() {

        if (!encoderIsAbsolute) {
            zeroEncoder();
        }

        // Makes the pivot motors brake
        setBrakeMode(true);

        if (RobotBase.isSimulation() && cfg.enableSimulation && cfg.simulationConfig != null) {
            simulation = new PivotSimulation("Pivot", cfg.simulationConfig, this);
            simulation.initialize();
        }
    }

    @Override
    public void periodic() {
        onPrePeriodic();
        onPeriodic();

        if (simulation != null) {
            simulation.setVoltage(lastAppliedVoltage);
            simulation.periodic();
        }

        onPostPeriodic();
    }

    protected void onPrePeriodic() {
    }

    protected void onPeriodic() {
        double currentRad = getAngleRad();
        if (manualMode) {
            applyVoltage(manualVoltage);
        } else if (enabled) {
            double clampedTarget = MathUtil.clamp(
                    targetAngleRad, Math.toRadians(cfg.minAngleDeg), Math.toRadians(cfg.maxAngleDeg));
            if (clampedTarget != targetAngleRad) {
                targetAngleRad = clampedTarget;
                pid.setGoal(targetAngleRad);
            }

            if (cfg.useSmartMotion) {
                // Gravity feedforward: kG * cos(theta)
                double gravityFF = cfg.kG * Math.cos(currentRad);

                // Convert target angle (radians) to motor rotations
                // motorRot = (angleRad * gearRatio) / 2PI
                double targetMotorRot = targetAngleRad * cfg.gearRatio / (2.0 * Math.PI);

                leader.setSmartPosition(targetMotorRot, gravityFF);

                lastAppliedVoltage = gravityFF;

                recordOutput("SmartMotion/TargetRot", targetMotorRot);
                recordOutput("SmartMotion/GravityFF", gravityFF);

            } else {
                double pidOut = pid.calculate(currentRad, targetAngleRad);
                double ffVolts = ff.calculate(targetAngleRad, pid.getSetpoint().velocity);
                double volts = pidOut + ffVolts;
                applyVoltage(volts);

                recordOutput("PIDOutput", pidOut);
                recordOutput("FFOutput", ffVolts);
                recordOutput("TotalVoltage", volts);
            }
        }
        // Logging
        recordOutput("Simulation Enabled", cfg.enableSimulation);
        recordOutput("angleDeg", getAngleDeg());
        recordOutput("target Deg", Math.toDegrees(targetAngleRad));
        recordOutput("at Target", atTarget());
        recordOutput("Is EncoderAbsolute", encoderIsAbsolute);
        recordOutput("Enabled", enabled);
        recordOutput("Last Applied Voltage", lastAppliedVoltage);

    }

    protected void onPostPeriodic() {
        // Unused hook
    }

    @Override
    public void stop() {
        onStop();
    }

    protected void onStop() {
        enabled = false;
        manualMode = false;
        applyVoltage(0.0);
    }

    public double getAngleRad() {
        if (cfg.encoder == null) {
            return 0.0;
        }
        double motorRot = cfg.encoder.getPositionMechanismRotations();
        double jointRot = motorRot / cfg.gearRatio;
        double rad = jointRot * 2.0 * Math.PI;
        if (cfg.encoderInverted) {
            rad = -rad;
        }
        rad += Math.toRadians(cfg.angleOffsetDeg);
        return rad;
    }

    public double getAngleDeg() {
        return Math.toDegrees(getAngleRad());
    }

    public void zeroEncoder() {
        if (cfg.encoder != null) {
            if (encoderIsAbsolute) {
                logWarn("Attempted to zero absolute encoder - this sets an offset, not true zero");
            }
            cfg.encoder.setPositionMechanismRotations(0);
        }
    }

    public void setTargetAngleDeg(double deg) {
        setTargetAngleRad(Math.toRadians(deg));
    }

    public void setTargetAngleRad(double rad) {
        targetAngleRad = rad;
        enabled = true;
        manualMode = false;
        pid.setGoal(rad);
    }

    public boolean atTarget() {
        return getAngleDeg() >= (Math.toDegrees(targetAngleRad) - cfg.toleranceDeg)
                && getAngleDeg() <= (Math.toDegrees(targetAngleRad) + cfg.toleranceDeg);
    }

    public double getTargetRad() {
        return targetAngleRad;
    }

    public double getTargetDeg() {
        return Math.toDegrees(targetAngleRad);
    }

    public void disable() {
        enabled = false;
        manualMode = false;
        applyVoltage(0.0);
    }

    public void setManualVoltage(double volts) {
        manualMode = true;
        enabled = false;
        manualVoltage = volts;
    }

    public void setBrakeMode(boolean brake) {
        leader.setBrakeMode(brake);
        for (var f : followers) {
            f.setBrakeMode(brake);
        }
    }

    private void applyVoltage(double volts) {
        double clamped = MathUtil.clamp(volts, -12.0, 12.0);
        lastAppliedVoltage = clamped;
        leader.setVoltage(clamped);
    }

    public void updatePID(double p, double i, double d) {
        pid.setPID(p, i, d);
    }

    public void updateFF(double kS, double kG, double kV, double kA) {
        this.ff = new ArmFeedforward(kS, kG, kV, kA);
    }

    /**
     * Returns true if using an absolute encoder
     */
    public boolean hasAbsoluteEncoder() {
        return encoderIsAbsolute;
    }

    @Override
    public Sendable log() {
        return (Sendable) null;
    }

    /**
     * Get the leader motor for simulation access.
     */
    public MotorWrapper getLeaderMotor() {
        return leader;
    }

    /**
     * Get the encoder for simulation access.
     */
    public EncoderWrapper getEncoder() {
        return cfg.encoder;
    }
}
