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

/**
 * Elevator subsystem for controlling a linear elevator mechanism.
 * Supports position and manual voltage control, simulation, and logging.
 */
public class Elevator extends AbsoluteSubsystem {

    /**
     * Configuration class for Elevator subsystem.
     * Holds all tunable parameters and hardware references.
     */
    public static class Config {

        // MOTORS AND ENCODER
        /** The leader motor wrapper. */
        public MotorWrapper leader;
        /** The follower motor wrappers. */
        public MotorWrapper[] followers = new MotorWrapper[0];
        /** The encoder wrapper. */
        public EncoderWrapper encoder;
        /** Whether the encoder is inverted. */
        public boolean encoderInverted = false;

        // MECHANICAL
        /** The gear ratio of the mechanism. */
        public double gearRatio = 1.0;
        /** The radius of the drum in meters. */
        public double drumRadiusMeters = 0.05;
        /** The minimum height in meters. */
        public double minHeightMeters = 0.0;
        /** The maximum height in meters. */
        public double maxHeightMeters = 1.0;
        /** The tolerated error in meters. */
        public double toleranceMeters = 0.02;
        /** The mass of the carriage in kg. */
        public double carriageMassKg = 5.0;

        // PID FF
        /** Proportional gain, Integral gain, Derivative gain. */
        public double kP = 0.0, kI = 0.0, kD = 0.0;
        /** Static gain, Gravity gain, Velocity gain, Acceleration gain. */
        public double kS = 0.0, kG = 0.0, kV = 0.0, kA = 0.0;

        // MOTION PROFILE
        /** The maximum velocity in meters per second. */
        public double maxVelocityMetersPerSec = 1.0;
        /** The maximum acceleration in meters per second squared. */
        public double maxAccelerationMetersPerSecSq = 2.0;

        // CONFIG
        /** The simulation configuration. */
        public ElevatorSimulation.ElevatorSimulationConfig simulationConfig = null;
        /** Whether simulation is enabled. */
        public boolean enableSimulation = true;
        /** Whether to use smart motion. */
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

        /**
         * Sets the drum radius.
         * @param meters the drum radius in meters
         * @return this config
         */
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

    /**
     * Constructs an Elevator with the given configuration.
     * @param config Elevator configuration object
     */
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

    /**
     * Default constructor is not supported. Use Elevator(new Config().withLeader(...)).
     */
    public Elevator() {
        throw new IllegalStateException("Use Elevator(new Config().withLeader(...)...)");
    }

    /**
     * Initializes the elevator subsystem, sets brake mode, and holds current position.
     */
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

    /**
     * Periodic update for the elevator subsystem. Handles simulation and control.
     */
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

    /**
     * Hook for logic before main periodic code.
     */
    protected void onPrePeriodic() {
    }

    /**
     * Hook for logic after main periodic code.
     */
    protected void onPostPeriodic() {
    }

    /**
     * Main periodic logic for elevator control and logging.
     */
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
     * @param meters Target height in meters.
     * @return Command that waits until target is reached
     */
    public Command setPosition(double meters) {
        return run(() -> {
            targetHeightMeters = MathUtil.clamp(meters, cfg.minHeightMeters, cfg.maxHeightMeters);
            manualMode = false;
        }).until(this::atTarget);
    }

    /**
     * Sets the target height in meters (no command).
     * @param meters Target height in meters.
     */
    public void setTargetHeight(double meters) {
        targetHeightMeters = MathUtil.clamp(meters, cfg.minHeightMeters, cfg.maxHeightMeters);
        manualMode = false;
    }

    /**
     * Stops the elevator and holds its current position.
     */
    @Override
    public void stop() {
        manualMode = false;
        // Hold current position when stopped
        targetHeightMeters = getHeightMeters();
    }

    /**
     * Sets manual voltage control mode.
     * @param volts Voltage to apply to the elevator motor.
     */
    public void setManualVoltage(double volts) {
        manualMode = true;
        manualVoltage = volts;
    }

    /**
     * Gets the current elevator height in meters.
     * @return Current height in meters.
     */
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

    /**
     * Checks if the elevator is at the target height.
     * @return True if at target, false otherwise.
     */
    public boolean atTarget() {
        return Math.abs(getHeightMeters() - targetHeightMeters) <= cfg.toleranceMeters;
    }

    /**
     * Sets brake mode for the elevator motors.
     * @param brake True to enable brake mode, false for coast.
     */
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

    /** 
     * Gets the leader motor.
     * @return the leader motor
     */
    public MotorWrapper getLeaderMotor() {
        return leader;
    }

    /** 
     * Gets the encoder.
     * @return the encoder
     */
    public EncoderWrapper getEncoder() {
        return cfg.encoder;
    }

    /** 
     * Gets the configuration used by this elevator.
     * @return the config
     */
    public Config getConfig() {
        return cfg;
    }

    /**
     * Gets the current target height in meters.
     * @return the target height in meters
     */
    public double getTargetHeightMeters() {
        return targetHeightMeters;
    }
}
