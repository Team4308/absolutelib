package ca.team4308.absolutelib.subsystems;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import edu.wpi.first.util.sendable.Sendable;

/**
 * Endeffector subsystem base class. Extend this class to create
 * endeffector subsystems (intakes, claws, etc.).
 * There is no default code you have to implement this class
 * start - Set the leader to a speed
 * stop set the leader to 0 
 */
public class EndEffector extends AbsoluteSubsystem {

	public static class Config {
		public MotorWrapper leader;
		public MotorWrapper[] followers = new MotorWrapper[0];
		public double angleOffsetDeg = 0.0;

		public boolean inverted = false;
		public double gearRatio = 1.0;
		public double kP = 0.0, kI = 0.0, kD = 0.0;
		public double kS = 0.0, kG = 0.0, kV = 0.0, kA = 0.0;
		public double minAngleDeg = -180.0;
		public double maxAngleDeg = 180.0;
		public double toleranceDeg = 2.0;

		public Config withLeader(MotorWrapper m) { this.leader = m; return this; }
		public Config withFollowers(MotorWrapper... m) { this.followers = m; return this; }
		public Config angleOffsetDeg(double offset){ angleOffsetDeg = offset; return this; }
		public Config inverted(boolean inv){inverted=inv;return this;}
	}

	private final Config config;

	public EndEffector(Config config) {
		this.config = config;
	}

	public final void initialize() {
		onInitialize();
	}

	protected void onInitialize() {
		config.leader.setInverted(config.inverted);
		for (MotorWrapper follower : config.followers) {
			follower.follow(config.leader);
		}
	}

	@Override
	public void periodic() {
		onPrePeriodic();
		onPeriodic();
		onPostPeriodic();
	}

	protected void onPeriodic() {
	}

	protected void onPrePeriodic() {
	}

	protected void onPostPeriodic() {
	}


	/**
	 * Stops the end effector.
	 */
	public void stop() {
		config.leader.set(0);
	}

	/**
	 * Sets the speed of the end effector.
	 * @param speed -1.0 to 1.0
	 */
	public void start(double speed) {
		config.leader.set(speed);
	}

	@Override
	public Sendable log() {
		return null;
	}
}

