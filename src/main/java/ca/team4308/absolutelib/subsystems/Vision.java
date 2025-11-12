package ca.team4308.absolutelib.subsystems;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.util.sendable.Sendable;

/**
 * Photon Vision, Not coded yet dont use.
 */
public class Vision extends AbsoluteSubsystem {

	public Vision() {
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
