package ca.team4308.absolutelib.subsystems;

import java.util.ArrayList;
import java.util.List;

import ca.team4308.absolutelib.subsystems.simulation.ElevatorSimulation;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends AbsoluteSubsystem {

    public static class Config {

        // MOTORS AND ENCODER
        public MotorWrapper leader;
        public MotorWrapper[] followers = new MotorWrapper[0];
        public EncoderWrapper encoder;
        public boolean encoderInverted = false;

        // MECHANICAL
        public double gearRatio = 1.0;
        public double drumRadiusMeters = 0.05;
        public double minHeightMeters = 0.0;
        public double maxHeightMeters = 1.0;
        public double toleranceMeters = 0.02;
        public double carriageMassKg = 5.0;

        // PID FF
        public double kP = 0.0, kI = 0.0, kD = 0.0;
        public double kS = 0.0, kG = 0.0, kV = 0.0, kA = 0.0;

        // MOTION PROFILE
        public double maxVelocityMetersPerSec = 1.0;
        public double maxAccelerationMetersPerSecSq = 2.0;

        // CONFIG
        public ElevatorSimulation.ElevatorSimulationConfig simulationConfig = null;
        public boolean enableSimulation = true;
        public boolean useSmartMotion = false;

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

        public Config gear(double ratio) {
            gearRatio = ratio;
            return this;
        }

        public Config drumRadius(double meters) {
            drumRadiusMeters = meters;
            return this;
        }

        public Config limits(double minMeters, double maxMeters) {
            minHeightMeters = minMeters;
            maxHeightMeters = maxMeters;
            return this;
        }

        public Config tolerance(double tolMeters) {
            toleranceMeters = tolMeters;
            return this;
        }

        public Config mass(double kg) {
            carriageMassKg = kg;
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

        public Config motion(double maxVel, double maxAccel) {
            maxVelocityMetersPerSec = maxVel;
            maxAccelerationMetersPerSecSq = maxAccel;
            return this;
        }

        public Config withSimulation(ElevatorSimulation.ElevatorSimulationConfig simCfg) {
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
    private ElevatorFeedforward ff;
    private final Config cfg;

    private double targetHeightMeters = 0.0;
    private boolean manualMode = false;
    private double manualVoltage = 0.0;

    private double lastAppliedVoltage = 0.0;

    // Simulation
    private ElevatorSimulation simulation;

    public Elevator(Config config) {
        this.cfg = config;
        this.leader = cfg.leader;
        for (MotorWrapper f : cfg.followers) {
            f.follow(leader);
            followers.add(f);
        }

        pid = new ProfiledPIDController(
                cfg.kP, cfg.kI, cfg.kD,
                new TrapezoidProfile.Constraints(
                        cfg.maxVelocityMetersPerSec,
                        cfg.maxAccelerationMetersPerSecSq));

        pid.setTolerance(cfg.toleranceMeters);
        ff = new ElevatorFeedforward(cfg.kS, cfg.kG, cfg.kV, cfg.kA);

        if (cfg.useSmartMotion) {
            // Smart Motion Config
            double cruiseRotPerSec = cfg.maxVelocityMetersPerSec / (2 * Math.PI * cfg.drumRadiusMeters) * cfg.gearRatio;
            double accelRotPerSecSq = cfg.maxAccelerationMetersPerSecSq / (2 * Math.PI * cfg.drumRadiusMeters) * cfg.gearRatio;

            MotorWrapper.UnifiedMotorConfig motorCfg = MotorWrapper.UnifiedMotorConfig.builder()
                    .kP(cfg.kP)
                    .kI(cfg.kI)
                    .kD(cfg.kD)
                    .kF(cfg.kV)
                    .motionCruiseVelocity(cruiseRotPerSec)
                    .motionAcceleration(accelRotPerSecSq);

            leader.applyMotorConfig(motorCfg);
        }
    }

    public Elevator() {
        throw new IllegalStateException("Use Elevator(new Config().withLeader(...)...)");
    }

    @Override
    protected void onInitialize() {
        // Brake mode by default
        setBrakeMode(true);

        // Hold current position on startup
        targetHeightMeters = getHeightMeters();

        if (RobotBase.isSimulation() && cfg.enableSimulation && cfg.simulationConfig != null) {
            simulation = new ElevatorSimulation("Elevator", cfg.simulationConfig, this);
            simulation.initialize();
        }
    }

    @Override
    public void periodic() {
        onPrePeriodic();
        onPeriodic();

        if (simulation != null && RobotBase.isSimulation() && cfg.enableSimulation) {
            simulation.setInputVoltage(lastAppliedVoltage);
            simulation.periodic();
        }

        onPostPeriodic();
    }

    protected void onPrePeriodic() {
    }

    protected void onPostPeriodic() {
    }

    protected void onPeriodic() {
        double currentMeters = getHeightMeters();

        if (manualMode) {
            applyVoltage(manualVoltage);
        } else {
            // Position control mode - always active
            if (cfg.useSmartMotion) {
                // Gravity FF is constant for elevator
                double gravityFF = cfg.kG;

                // Convert target meters to rotations
                // rotations = (meters / cyclumference) * gearRatio
                double circumference = 2.0 * Math.PI * cfg.drumRadiusMeters;
                double targetRot = (targetHeightMeters / circumference) * cfg.gearRatio;

                leader.setSmartPosition(targetRot, gravityFF);
                lastAppliedVoltage = gravityFF;

                recordOutput("SmartMotion/TargetRot", targetRot);
                recordOutput("SmartMotion/GravityFF", gravityFF);
            } else {
                double pidOut = pid.calculate(currentMeters, targetHeightMeters);
                double ffVolts = ff.calculate(pid.getSetpoint().velocity);
                double totalVolts = pidOut + ffVolts;

                applyVoltage(totalVolts);

                recordOutput("PIDOutput", pidOut);
                recordOutput("FFOutput", ffVolts);
                recordOutput("TotalVoltage", totalVolts);
            }
        }

        // Logging
        recordOutput("simulationEnabled", cfg.enableSimulation);
        recordOutput("heightMeters", currentMeters);
        recordOutput("targetMeters", targetHeightMeters);
        recordOutput("atTarget", atTarget());
        recordOutput("manualMode", manualMode);
        recordOutput("appliedVoltage", lastAppliedVoltage);
    }

    /**
     * Sets the target height in meters.
     *
     * @return Command that waits until target is reached
     */
    public Command setPosition(double meters) {
        return run(() -> {
            targetHeightMeters = MathUtil.clamp(meters, cfg.minHeightMeters, cfg.maxHeightMeters);
            manualMode = false;
        }).until(this::atTarget);
    }

    public void setTargetHeight(double meters) {
        targetHeightMeters = MathUtil.clamp(meters, cfg.minHeightMeters, cfg.maxHeightMeters);
        manualMode = false;
    }

    @Override
    public void stop() {
        manualMode = false;
        // Hold current position when stopped
        targetHeightMeters = getHeightMeters();
    }

    public void setManualVoltage(double volts) {
        manualMode = true;
        manualVoltage = volts;
    }

    public double getHeightMeters() {
        if (cfg.encoder == null) {
            return 0.0;
        }

        double motorRot = cfg.encoder.getPositionMechanismRotations();
        // rotations / gearRatio = drum rotations
        // drum rotations * circumference = meters
        double drumRot = motorRot / cfg.gearRatio;
        double circumference = 2.0 * Math.PI * cfg.drumRadiusMeters;
        double meters = drumRot * circumference;

        if (cfg.encoderInverted) {
            meters = -meters;
        }
        return meters;
    }

    public boolean atTarget() {
        return Math.abs(getHeightMeters() - targetHeightMeters) <= cfg.toleranceMeters;
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

    @Override
    public Sendable log() {
        return null;
    }

    // Getters for Simulation and external access
    public MotorWrapper getLeaderMotor() {
        return leader;
    }

    public EncoderWrapper getEncoder() {
        return cfg.encoder;
    }

    public Config getConfig() {
        return cfg;
    }

    public double getTargetHeightMeters() {
        return targetHeightMeters;
    }
}
