package ca.team4308.absolutelib.wrapper;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Unified motor controller wrapper, Doesnt matter the vendor.
 */
public class MotorWrapper {

    public enum MotorType {
        TALONFX,
        TALONSRX,
        VICTORSPX,
        SPARKMAX
    }

    private final MotorType type;
    private final int id;

    private final TalonFX talonFX;
    private final TalonSRX talonSRX;
    private final VictorSPX victorSPX;
    private final SparkMax sparkMax;
    private HashMap<Integer,MotorWrapper> motorWrappers = new HashMap();
    // Helpers
    private final DutyCycleOut fxDuty; 

    private final List<MotorWrapper> followers = new ArrayList<>();

    // Motion magic control (Phoenix 6)`
    private final MotionMagicVoltage fxMMVoltage = new MotionMagicVoltage(0);

    /**
     * Create a wrapper (SparkMax defaults to Brushless).
     */
    public MotorWrapper(MotorType type, int id) {
        this(type, id, SparkMax.MotorType.kBrushless);
    }

    

    /**
     * Create a wrapper. Use this overload to specify SparkMax motor type.
     * Sparkmax Motor type only needed for spark maxes.
     */
    public MotorWrapper(MotorType type, int id, SparkMax.MotorType sparkMotorType) {
        this.type = type;
        this.id = id;

        TalonFX fx = null;
        TalonSRX srx = null;
        VictorSPX spx = null;
        SparkMax sp = null;
        DutyCycleOut duty = null;
        
        switch (type) {
            case TALONFX -> {
                fx = new TalonFX(id);
                duty = new DutyCycleOut(0.0);
            }
            case TALONSRX -> srx = new TalonSRX(id);
            case VICTORSPX -> spx = new VictorSPX(id);
            case SPARKMAX -> sp = new SparkMax(id, sparkMotorType);
        }

        this.talonFX = fx;
        this.talonSRX = srx;
        this.victorSPX = spx;
        this.sparkMax = sp;
        this.fxDuty = duty;

        motorWrappers.put(id, this);

        
    }




    /**
     * Convenience factory for SparkMax with explicit motor type.
     */
    public static MotorWrapper sparkMax(int id, SparkMax.MotorType sparkMotorType) {
        return new MotorWrapper(MotorType.SPARKMAX, id, sparkMotorType);
    }

    public MotorType getType() { return type; }
    public int getId() { return id; }

    // Unified controls

    /**
     * Percent output [-1, 1].
     */
    public void set(double output) {
        switch (type) {
            case TALONFX -> talonFX.setControl(fxDuty.withOutput(output));
            case TALONSRX -> talonSRX.set(ControlMode.PercentOutput, output);
            case VICTORSPX -> victorSPX.set(ControlMode.PercentOutput, output);
            case SPARKMAX -> sparkMax.set(output);
        }
    }

    /**
     * Approximate voltage control (converts volts to percent based on 12V nominal).
     */
    public void setVoltage(double volts) {
        double pct = volts / 12.0;
        set(pct);
    }

    /**
     * Stop the motor (set output to 0).
     */
    public void stop() {
        set(0.0);
    }

    /**
     * READ BEFORE USING
     * Only SRX and Victor support .setInverted
     * TalonFX and SparkMAX require inversion via configuration
     */
    public void setInverted(boolean inverted) {
        switch (type) {
            case TALONFX -> {
                DriverStation.reportWarning("TALON FX Doesnt Support .setInverted, please update the config as of 2026", false);
            }
            case TALONSRX -> talonSRX.setInverted(inverted);
            case VICTORSPX -> victorSPX.setInverted(inverted);
            case SPARKMAX -> { 
                DriverStation.reportWarning("SparkMAX Doesnt Support .setInverted, please update the config as of 2026", false);
        }
        }
    }

    public boolean getInverted() {
        return switch (type) {
            case TALONFX -> talonFX.getInverted();
            case TALONSRX -> talonSRX.getInverted();
            case VICTORSPX -> victorSPX.getInverted();
            case SPARKMAX -> sparkMax.getInverted();
        };
    }

    /**
     * Make this motor follow a leader. Follows within the same vendor family.
     * Cross-vendor follow is not supported.
     */
    public void follow(MotorWrapper leader) {
        if (leader == null) throw new IllegalArgumentException("Leader cannot be null");

        // Same-type follow
        if (this.type == MotorType.TALONFX && leader.type == MotorType.TALONFX) {
            talonFX.setControl(new Follower(leader.talonFX.getDeviceID(), false));
            return;
        }
        if ((this.type == MotorType.TALONSRX || this.type == MotorType.VICTORSPX) &&
            (leader.type == MotorType.TALONSRX || leader.type == MotorType.VICTORSPX)) {
            BaseMotorController leaderBase = (BaseMotorController) leader.getRawController();
            if (this.type == MotorType.TALONSRX) {
                talonSRX.follow(leaderBase);
            } else {
                victorSPX.follow(leaderBase);
            }
            return;
        }
        if (this.type == MotorType.SPARKMAX && leader.type == MotorType.SPARKMAX) {
            SparkMaxConfig cf = new SparkMaxConfig();
            cf.follow( leader.sparkMax.getDeviceId());
            sparkMax.configure(cf,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            return;
        }

        throw new IllegalStateException("Cross-vendor follow not supported: " + this.type + " -> " + leader.type);
    }


    public TalonFX asTalonFX() { return talonFX; }
    public TalonSRX asTalonSRX() { return talonSRX; }
    public VictorSPX asVictorSPX() { return victorSPX; }

    // Rename-friendly access for REV SparkMax
    public SparkMax asSparkMax() { return sparkMax; }
    @Deprecated
    public SparkMax asCANSparkMax() { return sparkMax; }

    public boolean isTalonFX() { return type == MotorType.TALONFX; }
    public boolean isTalonSRX() { return type == MotorType.TALONSRX; }
    public boolean isVictorSPX() { return type == MotorType.VICTORSPX; }
    public boolean isSparkMax() { return type == MotorType.SPARKMAX; }

    /**
     * Returns the motor controller instance (TalonFX, TalonSRX, VictorSPX, or SparkMax).
     */
    public Object get() { return getRawController(); }

    /**
     * Set brake/coast (neutral/idle) mode.
     */
    public void setBrakeMode(boolean brake) {
        switch (type) {
            case TALONFX -> {
                MotorOutputConfigs cfg = new MotorOutputConfigs();
                cfg.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                talonFX.getConfigurator().apply(cfg);
            }
            case TALONSRX -> talonSRX.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
            case VICTORSPX -> victorSPX.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
            case SPARKMAX -> {
                SparkMaxConfig cfg = new SparkMaxConfig();
                cfg.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
                sparkMax.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            }
        }
    }

    /**
     * Set open-loop ramp (seconds from 0 to full).
     * Note: TalonFX P6 implementation is not provided here; unwrap to TalonFX for advanced configs.
     * DO NOT USE FOR SPARKMAX - SparkMax doesnt support Open-Loop-Ramp
     */
    public void setOpenLoopRamp(double seconds) {
        switch (type) {
            case TALONFX -> {

            }
            case TALONSRX -> talonSRX.configOpenloopRamp(seconds);
            case VICTORSPX -> victorSPX.configOpenloopRamp(seconds);
            case SPARKMAX ->  DriverStation.reportError("MotorWrapper: SparkMax doesnt support Open-Loop-Ramp", false);
        }
    }

    /**
     * Return the underlying vendor object (TalonFX, TalonSRX, VictorSPX, or SparkMax).
     */
    public Object getRawController() {
        return switch (type) {
            case TALONFX -> talonFX;
            case TALONSRX -> talonSRX;
            case VICTORSPX -> victorSPX;
            case SPARKMAX -> sparkMax;
        };
    }

    /**
     * Convenience unwrap to a requested vendor type. Returns null if the type does not match.
     */
    @SuppressWarnings("unchecked")
    public <T> T unwrap(Class<T> clazz) {
        Object raw = getRawController();
        return (clazz.isInstance(raw)) ? (T) raw : null;
    }

    // Unified motor configuration interface
    public interface MotorConfig {
        void apply(MotorWrapper wrapper);
    }

    /**
     * Builder-style unified motor configuration supporting common options and optional motion parameters.
     */
    public static class UnifiedMotorConfig implements MotorConfig {
        private boolean inverted = false;
        private Boolean brakeMode = null;
        private Double openLoopRamp = null;
        private Integer currentLimitAmps = null;          // (Spark / SRX if desired)
        private Double motionCruiseVelocity = null;       // native units per 100ms (SRX/Victor) or rotations/sec (FX/Spark)
        private Double motionAcceleration = null;         // native units per 100ms/sec (SRX/Victor) or rotations/sec^2 (FX/Spark)
        private Double motionJerk = null;                 // (Phoenix6 optional)
        private Double kP = null, kI = null, kD = null, kF = null;

        public static UnifiedMotorConfig builder() { return new UnifiedMotorConfig(); }

        public UnifiedMotorConfig inverted(boolean v) { this.inverted = v; return this; }
        public UnifiedMotorConfig brakeMode(boolean brake) { this.brakeMode = brake; return this; }
        public UnifiedMotorConfig openLoopRamp(double seconds) { this.openLoopRamp = seconds; return this; }
        public UnifiedMotorConfig currentLimit(int amps) { this.currentLimitAmps = amps; return this; }
        public UnifiedMotorConfig motionCruiseVelocity(double v) { this.motionCruiseVelocity = v; return this; }
        public UnifiedMotorConfig motionAcceleration(double a) { this.motionAcceleration = a; return this; }
        public UnifiedMotorConfig motionJerk(double j) { this.motionJerk = j; return this; }
        public UnifiedMotorConfig kP(double v){ this.kP=v; return this; }
        public UnifiedMotorConfig kI(double v){ this.kI=v; return this; }
        public UnifiedMotorConfig kD(double v){ this.kD=v; return this; }
        public UnifiedMotorConfig kF(double v){ this.kF=v; return this; }

        @Override
        public void apply(MotorWrapper w) {
            w.setInverted(inverted);
            if (brakeMode != null) w.setBrakeMode(brakeMode);
            if (openLoopRamp != null) w.setOpenLoopRamp(openLoopRamp);

            if (w.isTalonFX()) {
                Slot0Configs slot0 = new Slot0Configs();
                if (kP != null) slot0.kP = kP;
                if (kI != null) slot0.kI = kI;
                if (kD != null) slot0.kD = kD;
                if (kF != null) slot0.kV = kF; // Treat feedforward as kV
                w.asTalonFX().getConfigurator().apply(slot0);
            } else if (w.isTalonSRX()) {
                if (kP != null) w.asTalonSRX().config_kP(0, kP);
                if (kI != null) w.asTalonSRX().config_kI(0, kI);
                if (kD != null) w.asTalonSRX().config_kD(0, kD);
                if (kF != null) w.asTalonSRX().config_kF(0, kF);
            } else if (w.isVictorSPX()) {
                if (kP != null) w.asVictorSPX().config_kP(0, kP);
                if (kI != null) w.asVictorSPX().config_kI(0, kI);
                if (kD != null) w.asVictorSPX().config_kD(0, kD);
                if (kF != null) w.asVictorSPX().config_kF(0, kF);
            } else if (w.isSparkMax()) {
                
            }

            // Motion profiles
            if (motionCruiseVelocity != null && motionAcceleration != null) {
                if (w.isTalonFX()) {
                    w.configureMotionMagic(motionCruiseVelocity, motionAcceleration, motionJerk);
                } else if (w.isTalonSRX() || w.isVictorSPX()) {
                    w.configureMotionMagic(motionCruiseVelocity, motionAcceleration, null);
                } else if (w.isSparkMax()) {
                    w.configureMaxMotion(motionCruiseVelocity, motionAcceleration);
                }
            }
        }
    }

    /** Apply a unified MotorConfig. */
    public void applyMotorConfig(MotorConfig config) {
        if (config != null) config.apply(this);
    }

    /** Add a follower that mirrors this motor's output using vendor follow. */
    public void addFollower(MotorWrapper follower) {
        if (follower == null) return;
        follower.follow(this);
        followers.add(follower);
    }

    // Motion Magic (Phoenix / CTRE) and MAXMotion (REV)

    /**
     * Configure Motion Magic parameters (TalonFX Phoenix 6 or TalonSRX/VictorSPX Phoenix 5).
     * Units:
     *  TalonFX: rotations/sec (cruise), rotations/sec^2 (accel), rotations/sec^3 (jerk optional)
     *  TalonSRX/VictorSPX: native sensor units per 100ms (velocity), per 100ms/sec (accel)
     */
    public void configureMotionMagic(double cruiseVelocity, double acceleration, Double optionalJerk) {
        if (isTalonFX()) {
            MotionMagicConfigs mm = new MotionMagicConfigs();
            mm.MotionMagicCruiseVelocity = cruiseVelocity;
            mm.MotionMagicAcceleration = acceleration;
            if (optionalJerk != null) mm.MotionMagicJerk = optionalJerk;
            talonFX.getConfigurator().apply(mm);
        } else if (isTalonSRX()) {
            asTalonSRX().configMotionCruiseVelocity((int) cruiseVelocity);
            asTalonSRX().configMotionAcceleration((int) acceleration);
        } else if (isVictorSPX()) {
            asVictorSPX().configMotionCruiseVelocity((int) cruiseVelocity);
            asVictorSPX().configMotionAcceleration((int) acceleration);
        }
    }

    /**
     * Set a Motion Magic position target.
     * TalonFX: position (rotations).
     * TalonSRX/VictorSPX: raw sensor units (caller must supply).
     */
    public void setMotionMagicPosition(double position) {
        if (isTalonFX()) {
            talonFX.setControl(fxMMVoltage.withPosition(position));
        } else if (isTalonSRX()) {
            talonSRX.set(ControlMode.MotionMagic, position);
        } else if (isVictorSPX()) {
            victorSPX.set(ControlMode.MotionMagic, position);
        } else {
        }
    }

    /**
     * Configure REV MAXMotion (SparkMax).
     * Units expected: rotations/sec (velocity), rotations/sec^2 (acceleration).
     */
    public void configureMaxMotion(double cruiseVelocity, double acceleration) {
        if (!isSparkMax()) return;
        SparkMaxConfig cfg = new SparkMaxConfig();
        // REV 2025 API: closedLoop.maxMotion.* (adjust if API changes)
        cfg.closedLoop.maxMotion.maxVelocity(cruiseVelocity);
        cfg.closedLoop.maxMotion.maxAcceleration(acceleration);
        sparkMax.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Set MAXMotion position target (rotations).
     */
    public void setMaxMotionPosition(double position) {
        if (!isSparkMax()) return;
        // Using new closed loop controller (assumes position units are rotations)
        sparkMax.getClosedLoopController().setReference(position, com.revrobotics.spark.SparkBase.ControlType.kMAXMotionPositionControl);
    }

    /**
     * Generic smart position setter choosing appropriate vendor motion mode.
     * Falls back to percent output if unsupported.
     */
    public void setSmartPosition(double position) {
        if (isTalonFX() || isTalonSRX() || isVictorSPX()) {
            setMotionMagicPosition(position);
        } else if (isSparkMax()) {
            setMaxMotionPosition(position);
        } else {
            // not supported
            set(0.0);
        }
    }

        public MotorWrapper getMotorbyID(int id){
            for (MotorWrapper mw : motorWrappers.values()){
                if (mw.getId() == id){
                    return mw;
                }
            }
            return null;
        }


}
