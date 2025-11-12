# AbsoluteLib

[![](https://jitpack.io/v/team4308/absolutelib.svg)](https://jitpack.io/#team4308/absolutelib)

Team 4308's FRC library providing wrappers, utilities, and helpers for building robots quickly.

Version: 2.0.0
Updated: Nov 12, 2025


## Installation

Using JitPack with Gradle:

```groovy
repositories {
    maven { url 'https://jitpack.io' }
}

dependencies {
    implementation 'ca.team4308:absolutelib:2.0.0'
}
```

If you use the GitHub coordinates:

```groovy
dependencies {
    implementation 'com.github.team4308:absolutelib:2.0.0'
}
```

## Features

- Subsystem base with logging and telemetry (AbsoluteSubsystem)
- PathPlanner integration with on-the-fly path generation (2025 API)
- Motor and encoder wrappers (CTRE, REV, duty-cycle, CANCoder)
- Simulation-ready patterns
- WPILib chooser helpers for runtime path selection
- Premade Subsystems
---


## Subsystem Usage Examples

### Pivot (Arm/Wrist)

```java
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.subsystems.Pivot;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmSubsystem extends AbsoluteSubsystem {
    private final Pivot pivot;

    public ArmSubsystem() {
        MotorWrapper leader = MotorWrapper.createTalonFX(10);
        MotorWrapper follower = MotorWrapper.createTalonFX(11);
        EncoderWrapper encoder = EncoderWrapper.createCANCoder(20);

        pivot = new Pivot(new Pivot.Config()
            .withLeader(leader)
            .withFollowers(follower)
            .withEncoder(encoder)
            .pid(0.5, 0.0, 0.1)
            .ff(0.1, 0.5, 0.0, 0.0)
            .gear(100.0)
            .limits(-90, 90)
            .tolerance(2.0)
            .inverted(false)
        );
        initialize();
    }

    @Override
    public void periodic() {
        runPeriodicWithHooks(() -> {
            pivot.periodic();
            SDAdd("angleDeg", pivot.getAngleDeg());
            SDAdd("targetDeg", pivot.getTargetAngleDeg());
            SDAdd("atTarget", pivot.atTarget());
            logThrottle("pivotStatus", 500, "Pivot running");
        });
    }

    @Override
    public Sendable log() { return null; }

    // Command factories
    public Command setAngle(double degrees) {
        return runOnce(() -> pivot.setTargetAngleDeg(degrees)).until(pivot::atTarget);
    }

    public Command stow() { return setAngle(0); }
    public Command score() { return setAngle(45); }
    public Command intake() { return setAngle(-30); }
}
```

With simulation:

```java
pivot = new Pivot(new Pivot.Config()
    .withLeader(leader)
    .withEncoder(encoder)
    // ...existing config...
    .withSimulation(new PivotSimulation.Config()
        .armLengthMeters(0.5)
        .armMassKg(5.0)
        .motorGearbox(DCMotor.getNEO(1))
        .gearing(100.0)
    )
    .enableSimulation(true)
);
```

---

### Elevator (with top and bottom beam breaks)

```java
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import ca.team4308.absolutelib.subsystems.Elevator;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends AbsoluteSubsystem {
    private final Elevator elevator;

    // Beam breaks (active low)
    private final DigitalInput bottomBeamBreak = new DigitalInput(0);
    private final DigitalInput topBeamBreak = new DigitalInput(1);   

    public ElevatorSubsystem() {
        MotorWrapper leader = MotorWrapper.createSparkMax(5, SparkMaxType.BRUSHLESS);
        MotorWrapper follower = MotorWrapper.createSparkMax(6, SparkMaxType.BRUSHLESS);
        EncoderWrapper encoder = leader.getIntegratedEncoder();

        elevator = new Elevator(new Elevator.Config()
            .withLeader(leader)
            .withFollowers(follower)
            .withEncoder(encoder)
            .pid(5.0, 0.0, 0.5)
            .ff(0.05, 0.8, 0.0, 0.0)
            .sprocketRadiusMeters(0.025)
            .gear(12.0)
            .limits(0.0, 1.5)
            .tolerance(0.02)
        );
        initialize();
        logOnce("init", "Elevator initialized");
    }



    private boolean atTopLimit() {
        return !topBeamBreak.get();
    }

    @Override
    public void periodic() {
        runPeriodicWithHooks(() -> {
            elevator.periodic();

            if (atTopLimit() && elevator.getHeightMeters() > 1.45) {
                elevator.setTargetHeightMeters(1.45);
                logThrottle("limit", 1000, "Top limit enforced");
            }


            SDAdd("heightMeters", elevator.getHeightMeters());
            SDAdd("targetMeters", elevator.getTargetHeightMeters());
            SDAdd("atTarget", elevator.atTarget());
            SDAdd("topLimit", atTopLimit());
        });
    }

    @Override
    public Sendable log() { return null; }

    public Command goToHeight(double meters) {
        return runOnce(() -> elevator.setTargetHeightMeters(meters)).until(elevator::atTarget);
    }

    public Command groundLevel() { return goToHeight(0.0); }
    public Command lowGoal() { return goToHeight(0.5); }
    public Command highGoal() { return goToHeight(1.4); }


}
```

---

## Path Following (PathPlanner 2025)

On-the-fly path creation with OnTheFlyPathing, updated for the 2025 PathPlanner API. PathPoint takes only position and rotation target; use waypointsFromPoses and GoalEndState to set final velocity/heading.

```java
import ca.team4308.absolutelib.path.OnTheFlyPathing;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// Constraints (record): max lin vel/accel, max ang vel/accel, voltage, unlimited flag
PathConstraints constraints = OnTheFlyPathing.constraints(3.5, 3.0);

// Build simple direct path
Pose2d start = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));
Pose2d end   = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0));
Rotation2d endHeading = Rotation2d.fromDegrees(180);

PathPlannerPath path = OnTheFlyPathing.direct(
    start,
    end,
    constraints,
    endHeading,
    0.0,        // end velocity m/s
    true        // prevent alliance flipping (coords already correct)
);

// Build variants and select via chooser
var variants = OnTheFlyPathing.variants(start, end, constraints, endHeading, true);
SendableChooser<String> chooser = OnTheFlyPathing.buildChooser(variants, "direct");

PathPlannerPath selected = OnTheFlyPathing.selected(chooser, variants);
```

To create curved recipes:

```java
PathPlannerPath arcLeft = OnTheFlyPathing.arcViaOffsetMidpoint(
    start, end, +0.75, constraints, endHeading, 0.0, true);

PathPlannerPath sCurve = OnTheFlyPathing.sCurve(
    start, end, +0.6, -0.6, constraints, endHeading, 0.0, true);
```

Constraints helpers:

```java
// Convenience (defaults angular limits to very large values and 12V)
PathConstraints base = OnTheFlyPathing.constraints(3.0, 3.0);

// Derive slow/fast sets (uses record ctor)
PathConstraints slow = new PathConstraints(
    Math.min(base.maxVelocityMPS(), 2.0),
    Math.min(base.maxAccelerationMPSSq(), 2.0),
    base.maxAngularVelocityRadPerSec(),
    base.maxAngularAccelerationRadPerSecSq(),
    base.nominalVoltageVolts(),
    false
);
PathConstraints fast = new PathConstraints(
    base.maxVelocityMPS() * 1.25,
    base.maxAccelerationMPSSq() * 1.25,
    base.maxAngularVelocityRadPerSec() * 1.25,
    base.maxAngularAccelerationRadPerSecSq() * 1.25,
    base.nominalVoltageVolts(),
    false
);
```

---

