# AbsoluteLib V2

AbsoluteLib is an FRC utility library for Team 4308. It provides reusable subsystems + wrappers with a focus on "same code in real + sim".

- Vendor install: `https://team4308.github.io/absolutelib/lib/absolutelib.json`
- Docs site: `https://team4308.github.io/absolutelib/`
- JavaDoc site: `https://team4308.github.io/absolutelib/docs/javadoc`

---

## Installation (WPILib Vendor JSON)

1. WPILib VS Code → `Ctrl+Shift+P`
2. `WPILib: Manage Vendor Libraries`
3. `Install new library (online)`
4. Paste:
   ```text
   https://team4308.github.io/absolutelib/lib/absolutelib.json
   ```

### Required Vendor Dependencies

Before building, install all required libraries:

- Rev Hardware Client
- AdvantageKit
- PathPlannerLib
- Phoenix6-Replay (**6 and 5**)
- PhotonLib
- REVLib
- Studica
- ThriftyLib
- YAGSL

> **Note:** Your build WILL fail if you don't add these vendordeps.

---

## What You Get

### Wrappers

- `MotorWrapper`: TalonFX (Phoenix6), TalonSRX/VictorSPX (Phoenix5), SparkMax (REV) unified API
- `EncoderWrapper`: Unified encoder access (CANCoder, SparkMax encoder, etc.)

### Subsystems

- `Arm`: Multi-joint arm with IK support (2+ DOF arms with inverse kinematics)
- `Pivot`: Single-joint rotational control (PID + FF, optional Smart Motion)
- `Elevator`: Linear elevator control (PID + FF, optional Smart Motion)
- `EndEffector`: Base class for intakes, claws, and manipulators

### Simulation

- `ArmSimulation`: Physics sim for multi-DOF arms
- `PivotSimulation`: Physics sim via `SingleJointedArmSim`
- `ElevatorSimulation`: Physics sim via `ElevatorSim`

### Vision / LEDs

- `Vision`: PhotonVision wrapper + multi-camera support for pose estimation
- `leds/*`: Addressable LED patterns and simulation helpers

### Trajectory System (2026 REBUILT)

- `TrajectorySolver`: Complete trajectory solving for turret shooters with obstacle avoidance
- `ShooterSystem`: Integrated state machine for managing shots, tracking, and fallback strategies
- `FlywheelGenerator`: Automated flywheel configuration generation and optimization
- `FlywheelSimulator`: Physics-based flywheel and ball exit velocity simulation
- `ProjectileMotion`: Projectile physics with air resistance, drag, and Magnus effect
- `ObstacleConfig`: Collision-aware field obstacle definition (e.g., 2026 hub)
- `SolverConstants`: Runtime-tunable constants for all solver behavior
- `GamePieces`: Predefined game pieces including 2026 REBUILT ball

### Example Code

Under `./example/example-2026-Imported` you can find full robot code for all subsystems + simulation.

---

## Full Simulation: How It Works

AbsoluteLib simulations follow this pattern:

1. **Subsystem computes output** (PID + FF) → produces a voltage or percent output
2. Subsystem **applies output to motor** (real hardware OR sim state)
3. In SIM: subsystem passes the *same computed voltage* into the simulation object
4. Simulation updates position/velocity and writes them back into:
   - Encoder sim position
   - Motor controller sim state (when supported)

---

## Subsystem Examples

### MotorWrapper (Unified API)

```java
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;

public class ShooterIO {
    // Control TalonFX, SparkMax, or TalonSRX with the same API
    private final MotorWrapper leader = new MotorWrapper(MotorType.TALONFX, 10);
    private final MotorWrapper feeder = new MotorWrapper(MotorType.SPARKMAX, 11);

    public void runVolts(double volts) {
        leader.setVoltage(volts);
    }

    public void stop() {
        leader.stop();
        feeder.stop();
    }
}
```

---

### Pivot Subsystem (Arm/Wrist)

Single-joint rotational subsystem with PID control and simulation.

```java
import ca.team4308.absolutelib.subsystems.Pivot;
import ca.team4308.absolutelib.subsystems.simulation.PivotSimulation;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import edu.wpi.first.math.system.plant.DCMotor;

public class Wrist extends Pivot {
    public Wrist() {
        super(new Pivot.Config()
            .withLeader(new MotorWrapper(MotorWrapper.MotorType.TALONFX, 15))
            .withEncoder(EncoderWrapper.canCoder(20, 1.0)) // 1.0 = ratio derived from gearbox
            .gear(50.0)
            .limits(-90, 90)
            .pid(0.02, 0.0, 0.0)
            .ff(0.0, 0.15, 0.0, 0.0)
            .enableSimulation(true)
            .withSimulation(new PivotSimulation.Config()
                .gearbox(DCMotor.getFalcon500(1), 1)
                .gearRatio(50.0)
                .armLength(0.30)
                .armMass(2.0)
                .limits(Math.toRadians(-90), Math.toRadians(90))
                .startAngle(0.0)
            )
        );
    }

    public Command goToStow() { return setPosition(0.0); }
    public Command goToScore() { return setPosition(45.0); }
}
```

---

### Arm Subsystem (Multi-Joint IK)

Multi-DOF arm with inverse kinematics for Cartesian (x,y) positioning.

```java
import ca.team4308.absolutelib.subsystems.Arm;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;

public class ExampleArm {
    private final Arm arm = new Arm();
    private final Arm.Joint shoulder;

    public ExampleArm() {
        // Shoulder setup
        MotorWrapper shoulderMotor = new MotorWrapper(MotorType.TALONFX, 30);
        EncoderWrapper shoulderEncoder = EncoderWrapper.ofMechanismRotations(
            shoulderMotor::getPosition, 
            val -> shoulderMotor.asTalonFX().setPosition(val),
            1.0 / Math.PI // Conversion factor
        );

        Arm.JointConfig shoulderConfig = Arm.JointConfig.builder()
            .minAngleRad(-Math.PI / 2)
            .maxAngleRad(Math.PI / 2)
            .metersToRadians(2 * Math.PI)
            .linkLengthMeters(1.0)
            .build();

        shoulder = arm.addJoint(shoulderMotor, null, shoulderEncoder, shoulderConfig);
        shoulder.setPositionPID(32, 0, 0);
        shoulder.setFeedforwardGains(0.1, 0.1, 0.1, 0.1);

        // ... Add Elbow joint similarly ...

        arm.enableSimulation(true);
        arm.initialize();
    }

    // Modern IK control: Move end-effector to (x,y)
    public Command moveToPoint(double x, double y) {
        return runOnce(() -> arm.setGoalPose(x, y));
    }
}
```

---

### Vision (PhotonVision + Pose Estimation)

Seamless simulation-compatible vision integration.

```java
import ca.team4308.absolutelib.vision.Vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionSubsystem {
    private final Vision vision;

    public VisionSubsystem(SwerveDrive drive) {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        
        var frontCam = new Vision.VisionCamera(
            "FrontCam",
            layout,
            new Rotation3d(0, 0, 0),
            new Translation3d(0.30, 0.0, 0.20), // Camera offset from center
            VecBuilder.fill(0.8, 0.8, 2.0),    // Single tag trust
            VecBuilder.fill(0.3, 0.3, 1.0)     // Multi-tag trust
        );

        vision = new Vision(drive::getPose, drive.field, layout, frontCam);
    }

    // Call in periodic to update odometry
    public void updatePose(SwerveDrive drive) {
        vision.updatePoseEstimation(drive);
        vision.updateVisionField(); // Updates SmartDashboard field
    }
}
```

---

## Trajectory System (2026 REBUILT)

The layout for 2026 includes a powerful `ShooterSystem` state machine that integrates the `TrajectorySolver` with lookup tables and real-time physics.

### Shooter System Implementation

```java
import ca.team4308.absolutelib.math.trajectories.*;
import ca.team4308.absolutelib.math.trajectories.shooter.*;

public class Shooter {
    private final ShooterSystem shooterSystem;
    private final TrajectorySolver solver;

    public Shooter() {
        // 1. Configure Solver
        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.defaults()
            .toBuilder()
            .minPitchDegrees(47.5)
            .maxPitchDegrees(82.5)
            .build();
            
        solver = new TrajectorySolver(GamePieces.REBUILT_2026_BALL, solverConfig);
        // SWEEP mode finds best angle by testing candidates (thorough)
        // ITERATIVE mode is faster but less robust for complex obstacles
        solver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

        // 2. Configure Shooter Limits
        ShooterConfig config = ShooterConfig.builder()
            .pitchLimits(47.5, 82.5)
            .rpmLimits(0, 6000)
            .distanceLimits(0.5, 12.0)
            .safetyMaxExitVelocity(30.0)
            .build();

        // 3. Define Lookup Table (Fallback)
        ShotLookupTable table = new ShotLookupTable()
            .addEntry(1.0, 78.0, 1000)
            .addEntry(2.0, 72.0, 1300)
            .addEntry(4.0, 59.0, 2100)
            .addEntry(6.0, 52.0, 2700);

        // 4. Initialize System
        shooterSystem = new ShooterSystem(config, table, solver);
        shooterSystem.setMode(ShotMode.SOLVER_WITH_LOOKUP_FALLBACK);
    }
    
    public void periodic() {
        // Calculate shot based on current robot state
        ShotParameters shot = shooterSystem.calculate(
            distanceMeters, 
            currentFlywheelRpm, 
            robotVx, robotVy, 
            robotYawRadians
        );

        if (shot.valid) {
            // Apply shot.pitchDegrees and shot.rpm
        }
    }
}
```

### Obstacle Avoidance

The 2026 solver can be configured to avoid field structures (like the Hub).

```java
// Define Obstacle
ObstacleConfig hub = ObstacleConfig.builder()
    .position(4.03, 4.0)
    .baseSize(1.19)
    .wallHeight(1.83)
    .build();

// Add to Shot Input
ShotInput input = ShotInput.builder()
    .shooterPositionMeters(x, y, z)
    .targetPositionMeters(tx, ty, tz)
    .addObstacle(hub)
    .collisionCheckEnabled(true)
    .build();
```

### Tuning & Debugging

All physics constants are runtime-tunable via `SolverConstants`:

```java
SolverConstants.setVelocityBufferMultiplier(1.2); // 20% extra velocity buffer
SolverConstants.setDragCompensationMultiplier(1.0); // Air resistance factor
```

Enable debug logging to see exactly why shots are chosen or rejected:

```java
solver.setDebugEnabled(true);
// Logs accepted/rejected candidates, physics calculations, and flight paths
```

---

## Updating AbsoluteLib in Your Robot Project

When a new version of AbsoluteLib is released, follow these steps to update:

### Option 1: Automatic (Recommended)

1. WPILib VS Code → `Ctrl+Shift+P`
2. `WPILib: Manage Vendor Libraries`
3. `Check for updates (online)`
4. If AbsoluteLib shows an update, accept it.

This works because the vendor JSON URL points to the latest published version.

### Option 2: Manual Re-install

1. WPILib VS Code → `Ctrl+Shift+P`
2. `WPILib: Manage Vendor Libraries`
3. `Install new library (online)`
4. Paste:
   ```text
   https://team4308.github.io/absolutelib/lib/absolutelib.json
   ```
5. If prompted to replace the existing version, confirm.
6. Rebuild your project (`Ctrl+Shift+P` → `WPILib: Build Robot Code`).

### Option 3: Direct JSON Edit

1. Open `vendordeps/absolutelib.json` in your robot project.
2. Update the `"version"` fields to the new version number.
3. Update the `javaDependencies[0].version` to match.
4. Rebuild.

> **Tip:** After updating, always do a clean build (`./gradlew clean build`) to avoid stale cached artifacts.

---

## Releasing a New Version (For Maintainers)

To publish a new release of AbsoluteLib:

```bat
scripts\release.bat
```

This single script handles the entire release process:

1. **Safety checks** — verifies you're on `master`, inside the repo, and required files exist
2. **Version bump** — prompts for a new version and auto-updates `gradle.properties` and `absolutelib.json`
3. **Backup** — creates a timestamped local backup in `.git-backup-cache/`
4. **Commit & push master** — stages, commits, and pushes to the `master` branch
5. **Gradle build** — runs `publishPagesRepo` to generate Maven artifacts and Javadoc
6. **Site staging** — assembles the full GitHub Pages site (docs + Maven repo + Javadoc + vendor JSON)
7. **Deploy gh-pages** — force-pushes the built site to the `gh-pages` branch

No other scripts need to be run — `release.bat` replaces the old `publish.bat` and `build-pages-site.bat` scripts.

---

## Updating Libraries and WPILib Versions

1. **WPILib** — Go into the `build.gradle` file and change `wpilibVersion` to the year. You may also have to update `WPILibRepositoriesPlugin` only if they rework vendors.
2. **Libraries** — Also in the `build.gradle` file, scroll down to the `dependencies` block and update each package as needed.
