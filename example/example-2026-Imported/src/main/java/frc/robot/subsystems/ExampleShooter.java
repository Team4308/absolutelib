package frc.robot.subsystems;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;

public class ExampleShooter extends AbsoluteSubsystem{
// I need to finish ts later im tired asf
    public void calculateShot() {
        TrajectorySolver solver = TrajectorySolver.forGame2026();

        ShotInput input = ShotInput.builder()
            .shooterPositionMeters(2.0, 3.0, 0.6)
            .targetPositionMeters(8.0, 6.0, 2.5)
            .preferHighArc(true)
            .build();

        TrajectoryResult result = solver.solve(input);

        if (result.isSuccess()) {
            double pitch = result.getPitchAngleDegrees();
            double rpm = result.getRecommendedRpm();
            FlywheelConfig flywheel = result.getRecommendedFlywheel();
        }
    }
    
}
