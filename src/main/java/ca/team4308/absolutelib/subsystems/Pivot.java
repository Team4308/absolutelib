package ca.team4308.absolutelib.subsystems;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.MathUtil;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
// Added for simulation
import edu.wpi.first.wpilibj.RobotBase;
import ca.team4308.absolutelib.subsystems.simulation.PivotSimulation;

public class Pivot extends AbsoluteSubsystem {

	public static class Config {
		public MotorWrapper leader;
		public MotorWrapper[] followers = new MotorWrapper[0];
		public EncoderWrapper encoder;
		public boolean encoderInverted = false;
		public double angleOffsetDeg = 0.0;

		public boolean inverted = false;
		public double gearRatio = 1.0;
		public double kP = 0.0, kI = 0.0, kD = 0.0;
		public double kS = 0.0, kG = 0.0, kV = 0.0, kA = 0.0;
		public double minAngleDeg = -180.0;
		public double maxAngleDeg = 180.0;
		public double toleranceDeg = 2.0;

		// Added simulation config
		public PivotSimulation.Config simulationConfig = null;
		public boolean enableSimulation = true;

		public Config withLeader(MotorWrapper m) { this.leader = m; return this; }
		public Config withFollowers(MotorWrapper... m) { this.followers = m; return this; }
		public Config withEncoder(EncoderWrapper e) { this.encoder = e; return this; }
		public Config encoderInverted(boolean inv){ encoderInverted = inv; return this; }
		public Config angleOffsetDeg(double offset){ angleOffsetDeg = offset; return this; }
		public Config pid(double p,double i,double d){kP=p;kI=i;kD=d;return this;}
		public Config ff(double s,double g,double v,double a){kS=s;kG=g;kV=v;kA=a;return this;}
		public Config gear(double ratio){gearRatio=ratio;return this;}
		public Config limits(double minDeg,double maxDeg){minAngleDeg=minDeg;maxAngleDeg=maxDeg;return this;}
		public Config tolerance(double tolDeg){toleranceDeg=tolDeg;return this;}
		public Config inverted(boolean inv){inverted=inv;return this;}
		
		// Simulation builder methods
		public Config withSimulation(PivotSimulation.Config simCfg){ 
			this.simulationConfig = simCfg; 
			return this; 
		}
		public Config enableSimulation(boolean enable){ 
			this.enableSimulation = enable; 
			return this; 
		}
	}

	private final MotorWrapper leader;
	private final List<MotorWrapper> followers = new ArrayList<>();
	private final PIDController pid;
	private ArmFeedforward ff; 
	private final Config cfg;

	// Added state
	private double targetAngleRad = 0.0;
	private boolean enabled = false;
	private boolean manualMode = false;
	private double manualVoltage = 0.0;

	// Simulation integration
	private PivotSimulation simulation = null;
	private double lastAppliedVoltage = 0.0;

	// Track encoder type for proper simulation handling
	private final boolean encoderIsAbsolute;
	private double absoluteEncoderInitialOffset = 0.0;

	public Pivot(Config config) {
		this.cfg = config;
		leader = cfg.leader;
		for (MotorWrapper f : cfg.followers) {
			f.follow(leader);
			followers.add(f);
		}
		pid = new PIDController(cfg.kP, cfg.kI, cfg.kD);
		pid.setTolerance(Math.toRadians(cfg.toleranceDeg));
		ff = new ArmFeedforward(cfg.kS, cfg.kG, cfg.kV, cfg.kA);

		encoderIsAbsolute = (cfg.encoder != null) && cfg.encoder.isAbsolute();
		if (encoderIsAbsolute && cfg.encoder != null) {
			absoluteEncoderInitialOffset = cfg.encoder.getPositionMeters();
		}

		// Only initialize simulation, do not run any loop here
		if (RobotBase.isSimulation() && cfg.enableSimulation) {
			initSimulation();
		}
	}

	public Pivot() {
		throw new IllegalStateException("Use Pivot(new Config().withLeader(...).withEncoder(...))");
	}

	/** Initialize simulation (called automatically if enableSimulation=true in sim) */
	private void initSimulation() {
		if (simulation != null) return; // already initialized

		PivotSimulation.Config simCfg = cfg.simulationConfig;
		if (simCfg == null) {
			// Auto-generate sim config from pivot config
			simCfg = PivotSimulation.Config.fromPivotConfig(
				cfg, 
				edu.wpi.first.math.system.plant.DCMotor.getNEO(1), 
				1 + cfg.followers.length, 
				0.5, // default arm length
				5.0  // default arm mass
			);
			logWarn("Auto-generated simulation config with defaults. Use withSimulation() for accuracy.");
		}

		simulation = new PivotSimulation(getName() != null ? getName() : "pivot", simCfg);
		simulation.initialize();
		logInfo("Simulation initialized and linked to Pivot (encoder type: " + 
			(encoderIsAbsolute ? "absolute" : "relative") + ")");
	}

	/** Manually set simulation config (useful for tuning) */
	public void setSimulation(PivotSimulation sim) {
		this.simulation = sim;
	}

	/** Get the linked simulation (null if not in sim mode or disabled) */
	public PivotSimulation getSimulation() {
		return simulation;
	}

	// Lifecycle
	public final void initialize() {
		onInitialize();
	}

	protected void onInitialize() {
		// For relative encoders, zero them
		// For absolute encoders, keep their position
		if (!encoderIsAbsolute) {
			zeroEncoder();
		}
		setBrakeMode(true);
		if (simulation != null) {
			simulation.initialize();
			// Initialize sim to match current encoder position
			if (cfg.encoder != null) {
				double currentAngle = getAngleRad();
				simulation.setSimulationPosition(currentAngle);
			}
		}
	}

	@Override
	public void periodic() {
		onPrePeriodic();
		onPeriodic();

		// Only update simulation if present, do not call periodic recursively
		if (simulation != null) {
			simulation.setVoltage(lastAppliedVoltage);
			simulation.periodic();
		}

		onPostPeriodic();
	}

	protected void onPrePeriodic() {}

	protected void onPeriodic() {
		double currentRad = getAngleRad();
		if (manualMode) {
			applyVoltage(manualVoltage);
		} else if (enabled) {
			double clampedTarget = MathUtil.clamp(
				targetAngleRad, Math.toRadians(cfg.minAngleDeg), Math.toRadians(cfg.maxAngleDeg)
			);
			if (clampedTarget != targetAngleRad) targetAngleRad = clampedTarget;

			double pidOut = pid.calculate(currentRad, targetAngleRad);
			double ffVolts = ff.calculate(targetAngleRad, 0.0); // hold by default
			double volts = pidOut + ffVolts;
			applyVoltage(volts);
		}
		// Telemetry
		SDAdd("angleDeg", getAngleDeg());
		SDAdd("targetDeg", Math.toDegrees(targetAngleRad));
		SDAdd("atTarget", atTarget());
		SDAdd("encoderAbsolute", encoderIsAbsolute);
	}

	protected void onPostPeriodic() {}

	@Override
	public void stop() {
		onStop();
		if (simulation != null) {
			simulation.stop();
		}
	}

	protected void onStop() {
		enabled = false;
		manualMode = false;
		applyVoltage(0.0);
	}

	// Angle access using EncoderWrapper
	public double getAngleRad() {
		if (cfg.encoder == null) return 0.0;
		double motorRot = cfg.encoder.getPositionMeters(); 
		double jointRot = motorRot / cfg.gearRatio;
		double rad = jointRot * 2.0 * Math.PI;
		if (cfg.encoderInverted) rad = -rad;
		rad += Math.toRadians(cfg.angleOffsetDeg);
		return rad;
	}

	public double getAngleDeg() { return Math.toDegrees(getAngleRad()); }

	public void zeroEncoder() {
		if (cfg.encoder != null) {
			if (encoderIsAbsolute) {
				logWarn("Attempted to zero absolute encoder - this sets an offset, not true zero");
			}
			cfg.encoder.setPositionMeters(0);
		}
		if (simulation != null) simulation.setSimulationPosition(0.0);
	}

	// Control helpers
	public void setTargetAngleDeg(double deg) { setTargetAngleRad(Math.toRadians(deg)); }
	public void setTargetAngleRad(double rad) {
		targetAngleRad = rad;
		enabled = true;
		manualMode = false;
		pid.setSetpoint(rad);
	}

	public boolean atTarget() { return enabled && pid.atSetpoint(); }
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
		for (var f : followers) f.setBrakeMode(brake);
	}

	private void applyVoltage(double volts) {
		double clamped = MathUtil.clamp(volts, -12.0, 12.0);
		lastAppliedVoltage = clamped;
		leader.setVoltage(clamped);
	}

	public void updatePID(double p,double i,double d){
		pid.setP(p); pid.setI(i); pid.setD(d);
	}

	public void updateFF(double kS,double kG,double kV,double kA){
		this.ff = new ArmFeedforward(kS, kG, kV, kA);
	}

	/** Returns true if using an absolute encoder */
	public boolean hasAbsoluteEncoder() {
		return encoderIsAbsolute;
	}

	@Override
	public Sendable log() {
		return (Sendable) null;
	}
}
