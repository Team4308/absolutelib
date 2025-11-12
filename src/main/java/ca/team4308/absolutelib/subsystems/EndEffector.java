package ca.team4308.absolutelib.subsystems;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import edu.wpi.first.util.sendable.Sendable;

/**
 * Endeffector subsystem base class. Extend this class to create
 * endeffector subsystems (intakes, claws, etc.).
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

	public EndEffector() {

	}

	public final void initialize() {
		onInitialize();
	}

	protected void onInitialize() {
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

	@Override
	public void stop() {
		onStop();
	}

	protected void onStop() {
	}

	@Override
	public Sendable log() {
		return null;
	}
}

