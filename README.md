# AbsoluteLib V2

[![](https://jitpack.io/v/team4308/absolutelib.svg)](https://jitpack.io/#team4308/absolutelib)
![Java CI](https://github.com/team4308/absolutelib/actions/workflows/gradle.yml/badge.svg)
![License](https://img.shields.io/github/license/team4308/absolutelib)

AbsoluteLib is an FRC utility library for Team 4308 providing reusable subsystems, math helpers, and adapters around WPILib and common vendor APIs.

---

## Installation

### WPILib Vendor Library (recommended)
1. Open your robot project in VS Code with the WPILib extension.
2. Press <kbd>Ctrl+Shift+P</kbd> → **WPILib: Manage Vendor Libraries**.
3. Choose **Install new library (online)**.
4. When prompted for a URL, enter:
   ```text
   https://team4308.github.io/absolutelib/lib/absolutelib.json
   ```
5. Save and let Gradle refresh.

The library will be added automatically via the vendordep. If you need to reference it manually, add the following to `build.gradle`:
```gradle
dependencies {
    implementation "ca.team4308:absolutelib-java:1.1.3"
}
```

### Direct JitPack Dependency (non‑FRC projects)
```gradle
repositories {
    maven { url "https://jitpack.io" }
}

dependencies {
    implementation "com.github.Team4308:absolutelib:1.1.3"
}
```

---

## Features
- Subsystem base with logging and telemetry (`AbsoluteSubsystem`).
- PathPlanner integration with on‑the‑fly path generation (2025 API).
- Motor and encoder wrappers for CTRE, REV, duty‑cycle, CANCoder, etc.
- Simulation‑ready patterns.
- WPILib chooser helpers for runtime path selection.
- Pre‑made subsystems (Elevator, Pivot, Arm, …).
- PID auto‑tuning utilities (`SimpleTune`, `PIDOptimizer`).
- Music player for TalonFX motors.
- LED pattern generators.

---

## Example Usage (Importing from a New Project)
All examples assume you have added AbsoluteLib as a dependency as described above.

### Pivot Subsystem (Arm/Wrist)
```java
public class ArmPivot extends Pivot {
    public ArmPivot() {
        super(new Config()
                .withLeader(new MotorWrapper(MotorWrapper.MotorType.TALONFX, 10))
                .withEncoder(EncoderWrapper.canCoder(20, 1.0, 4096, 0.1)) // ID, Ratio, CPR, Diameter
                .pid(0.5, 0.0, 0.1)
                .ff(0.1, 0.5, 0.0, 0.0)
                .gear(100.0)
                .limits(-90, 90)
                .tolerance(2.0)
                .inverted(false)
                .enableSimulation(true)); // Enable built-in simulation
        initialize();
    }

    /**
     * Sets the target position for the pivot.
     * @param targetPosition The desired position in degrees.
     */
    public void setTargetPosition(double targetPosition) {
        setSetpoint(targetPosition);
    }

    /**
     * Returns the current position of the pivot.
     * @return The current position in degrees.
     */
    public double getCurrentPosition() {
        return getPosition();
    }
}
```

### Elevator Subsystem
```java
public class RobotElevator extends Elevator {
    public RobotElevator() {
        super(
            new MotorWrapper(MotorWrapper.MotorType.TALONFX, 10),
            null, // MotorConfig (optional)
            EncoderWrapper.canCoder(20, 1.0, 4096, 0.05), // ID, Ratio, CPR, Diameter
            ElevatorConfig.builder()
                .minHeightMeters(0.0)
                .maxHeightMeters(2.0)
                .drumDiameterMeters(0.05)
                .gearRatio(10.0)
                .maxVelocityMetersPerSec(1.5)
                .maxAccelerationMetersPerSecSq(2.0)
                .toleranceMeters(0.01)
                .enableSimulation(true) // Enable built-in simulation
                .build()
        );
        
        // Configure control gains
        setPositionPID(1.0, 0.0, 0.0);
        setFeedforwardGains(0.1, 0.5, 0.2, 0.05);
        
        initialize();
    }

    /**
     * Sets the target height for the elevator.
     * @param targetHeight The desired height in meters.
     */
    public void setTargetHeight(double targetHeight) {
        setSetpoint(targetHeight);
    }

    /**
     * Returns the current height of the elevator.
     * @return The current height in meters.
     */
    public double getCurrentHeight() {
        return getHeight();
    }
}

public class RobotElevator extends Elevator {
    public RobotElevator() {
        super(
            new MotorWrapper(MotorWrapper.MotorType.TALONFX, 10),
            null, // MotorConfig (optional)
            EncoderWrapper.canCoder(20, 1.0, 4096, 0.05), // ID, Ratio, CPR, Diameter
            ElevatorConfig.builder()
                .minHeightMeters(0.0)
                .maxHeightMeters(2.0)
                .drumDiameterMeters(0.05)
                .gearRatio(10.0)
                .maxVelocityMetersPerSec(1.5)
                .maxAccelerationMetersPerSecSq(2.0)
                .toleranceMeters(0.01)
                .enableSimulation(true) // Enable built-in simulation
                .build()
        );
        
        // Configure control gains
        setPositionPID(1.0, 0.0, 0.0);
        setFeedforwardGains(0.1, 0.5, 0.2, 0.05);
        
        initialize();
    }
}
```

### Multi‑Joint Arm with Inverse Kinematics
```java
package frc.robot.subsystems;

import ca.team4308.absolutelib.subsystems.Arm;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;

public class RobotArm extends Arm {
    public RobotArm() {
        // Enable built-in simulation
        enableSimulation(true);

        // Shoulder joint
        addJoint(
                new MotorWrapper(MotorWrapper.MotorType.TALONFX, 1),
                null,
                EncoderWrapper.canCoder(11, 1.0, 4096, 0.1),
                JointConfig.builder()
                        .minAngleRad(Math.toRadians(-90))
                        .maxAngleRad(Math.toRadians(90))
                        .linkLengthMeters(0.8)
                        .build()
        );
        // Elbow joint
        addJoint(
                new MotorWrapper(MotorWrapper.MotorType.TALONFX, 2),
                null,
                EncoderWrapper.canCoder(12, 1.0, 4096, 0.1),
                JointConfig.builder()
                        .minAngleRad(Math.toRadians(0))
                        .maxAngleRad(Math.toRadians(150))
                        .linkLengthMeters(0.6)
                        .build()
        );
        
        initialize();
    }

    /** Move the end‑effector to a Cartesian coordinate using the built‑in IK solver. */
    public void moveTo(double xMeters, double yMeters) {
        setGoalPose(xMeters, yMeters);
    }
}
```

### SimpleTune PID Auto‑Tuner
```java
import ca.team4308.absolutelib.pid.SimpleTune;
import ca.team4308.absolutelib.pid.SimpleTune.PIDResult;
import ca.team4308.absolutelib.wrapper.MotorWrapper;

public class TunerDemo {
    private final SimpleTune tuner = new SimpleTune(90.0, 2.0, 0.5);
    private final MotorWrapper motor = new MotorWrapper(MotorWrapper.MotorType.TALONFX, 4);

    public void periodic(double measurement) {
        double voltage = tuner.update(measurement);
        motor.setVoltage(voltage);
        if (tuner.isFinished()) {
            motor.setVoltage(0);
            PIDResult result = tuner.calculateConstants(SimpleTune.TuningRule.ZIEGLER_NICHOLS);
            System.out.println("Tuned PID: " + result);
        }
    }
}
```

### PIDOptimizer (Twiddle) Example
```java
import ca.team4308.absolutelib.pid.SimpleTune.PIDOptimizer;
import ca.team4308.absolutelib.pid.SimpleTune.PIDResult;

public class OptimizerDemo {
    private final PIDOptimizer optimizer = new PIDOptimizer(0.1, 0.0, 0.0);

    /** Call after each test run with the measured error score. */
    public void feedScore(double errorScore) {
        PIDResult next = optimizer.tune(errorScore);
        // Apply the new constants to your controller here
        System.out.println("Next constants: " + next);
    }
}
```

### Music Player (CHRP files)
```java
import ca.team4308.absolutelib.other.musicPlayer;
import com.ctre.phoenix.music.Orchestra;
import ca.team4308.absolutelib.wrapper.MotorWrapper;

public class MusicDemo {
    private final musicPlayer player;

    public MusicDemo(MotorWrapper leftLeader, MotorWrapper rightLeader) {
        Orchestra orchestra = new Orchestra();
        player = new musicPlayer(orchestra);
        player.addInstruments(new MotorWrapper[] { leftLeader, rightLeader });
    }

    public void playSong(String path) {
        player.loadSong(path); // e.g. "deploy/song.chrp"
        player.playSong();
    }
}
```

### Controls & Wrappers
```java
import ca.team4308.absolutelib.control.JoystickHelper;

public class ControlsDemo {
    private final JoystickHelper driver = new JoystickHelper(0);

    public double getSpeed() {
        return driver.getLeftY(); // deadband & scaling applied automatically
    }
}
```

### LED Patterns
```java
import ca.team4308.absolutelib.leds.Leds;
import ca.team4308.absolutelib.leds.Patterns;
import edu.wpi.first.wpilibj.util.Color;

public class LedDemo {
    private final Leds leds = new Leds(9, 60);

    public void setRainbow() {
        leds.setPattern(Patterns.rainbow());
    }

    public void setRed() {
        leds.setPattern(Patterns.solid(Color.kRed));
    }

    public void periodic() {
        leds.periodic();
    }
}
```

### Path Following (PathPlanner 2025)
```java
import ca.team4308.absolutelib.path.OnTheFlyPathing;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathDemo {
    public PathPlannerPath createPath() {
        PathConstraints constraints = OnTheFlyPathing.constraints(3.5, 3.0);
        Pose2d start = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));
        Pose2d end   = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(180));
        return OnTheFlyPathing.direct(
                start,
                end,
                constraints,
                Rotation2d.fromDegrees(180),
                0.0,        // end velocity m/s
                true        // prevent alliance flipping
        );
    }
}
```

---

## Contributing
Feel free to open issues or submit pull requests. Follow the standard GitHub workflow and ensure all tests pass before merging.

---

## License
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

---

## Maintainers
- **Team 4308** – <team4308@frc.org>

### Version bump
1. Update `gradle.properties` with the new version.
2. Update `absolutelib.json` to match the new version.
3. Update the README examples if the API changes.
4. Run `scripts\\release-absolutelib.bat` to automate committing, tagging, and publishing.
