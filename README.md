# AbsoluteLib

[![](https://jitpack.io/v/team4308/absolutelib.svg)](https://jitpack.io/#team4308/absolutelib)

AbsoluteLib is an FRC utility library for Team 4308 providing reusable subsystems, math helpers, and adapters around WPILib and common vendor APIs.
Updated: Nov 16, 2025

---

## Installation (Recommended: WPILib Vendor JSON)

AbsoluteLib is distributed as a WPILib vendor library. The vendor JSON is hosted on GitHub Pages and the artifacts are hosted on a public Maven repository that lives under GitHub Pages.

Use this for normal FRC robot projects.

1. Open your robot project in VS Code (WPILib extension installed).
2. Press `Ctrl+Shift+P` and run: `WPILib: Manage Vendor Libraries`.
3. Choose: `Install new library (online)`.
4. When prompted for a URL, enter:

   ```text
   https://team4308.github.io/absolutelib/absolutelib.json
   ```

5. Save and let Gradle refresh.

After installation, absolutelib will be included automatically via the vendordep. If you want to reference it explicitly in `build.gradle`:

```gradle
dependencies {
    implementation "ca.team4308:absolutelib-java:1.0.5"
    // ...your other dependencies...
}
```

### Maven Repository Details (Vendordep path)

The vendor JSON (`absolutelib.json`) currently references:

```json
"mavenUrls": [
  "https://team4308.github.io/absolutelib",
  "https://jitpack.io"
]
```

The self-hosted Maven repo (under GitHub Pages) exposes:

- Group ID: `ca.team4308`
- Artifact ID: `absolutelib-java`
- Version: `1.0.5`

The path on Pages is:

```text
https://team4308.github.io/absolutelib/ca/team4308/absolutelib-java/1.0.5/absolutelib-java-1.0.5.jar
```

If you want to consume this directly without the vendordep, you can add:

```gradle
repositories {
    mavenCentral()
    maven {
        url = uri("https://team4308.github.io/absolutelib")
    }
}

dependencies {
    implementation "ca.team4308:absolutelib-java:1.0.5"
}
```

---

## Alternative: Direct Gradle Dependency via JitPack

For non-FRC projects or where you do not want to use a WPILib vendordep, you can use JitPack.

Coordinates (see JitPack page for latest):

- Group ID: `com.github.Team4308`
- Artifact ID: `absolutelib`
- Version: `1.0.5` (or tag of your choice)

Add to your Gradle project:

```gradle
repositories {
    maven { url "https://jitpack.io" }
    // other repos...
}

dependencies {
    implementation "com.github.Team4308:absolutelib:1.0.5"
}
```

This bypasses the FRC vendordep mechanism entirely and pulls straight from JitPack.

---

## Maintainers: How to Update and Publish

This section is for people maintaining the library.

### 1. Version bump

1. Update the Gradle project version:

   ```properties
   // gradle.properties
   group=ca.team4308
   version=1.0.X
   ```

2. Update the vendordep JSON version and dependency:

   ```json
   // absolutelib.json
   "version": "1.0.X",
   "javaDependencies": [
     {
       "groupId": "ca.team4308",
       "artifactId": "absolutelib-java",
       "version": "1.0.X"
     }
   ]
   ```

3. Update the README examples to match the new version:

   ```gradle
   implementation "ca.team4308:absolutelib-java:1.0.X"
   implementation "com.github.Team4308:absolutelib:1.0.X"
   ```

You can also run `scripts\release-absolutelib.bat` from the repo root to automate these steps (it bumps versions, commits, tags, and pushes).

### 2. Publishing the Java artifact

There are two publishing paths:

- GitHub Packages (for internal use / debug)
- Self-hosted Maven under GitHub Pages (for vendordep)

#### GitHub Packages

From the `absolutelib` root:

```powershell
scripts\publish-github-packages.bat
```

The script:

- Prompts for your GitHub username and a PAT with `read:packages`, `write:packages`.
- Runs `gradlew verifyPublishCreds` and then `gradlew publish`.
- Uses the `GitHubPackages` repository block in `build.gradle` to upload.

If you get a 409 conflict, the version already exists; bump `version` in `gradle.properties` if you really need a new build.

#### Self-hosted Maven (GitHub Pages)

GitHub Actions generates and deploys the Maven repo automatically from `main`:

1. Push your changes to `main` (either manually or via `scripts\publish-latest.bat`):

   ```powershell
   scripts\publish-latest.bat
   ```

   This optionally runs `build-site` locally and commits/pushes your code.

2. The `.github/workflows/pages.yml` workflow runs on pushes to `main` and:

   - Calls:

     ```bash
     ./gradlew "-Pversion=${PUBLISH_VERSION}" "-PpagesPublish=true" publishPagesRepo
     ```

   - Writes the Maven repo to `build/pages-maven`.
   - Uploads `build/pages-maven` as the Pages artifact and deploys it.

GitHub Pages is configured to serve the `build/pages-maven` content, so:

- Vendor JSON: `https://team4308.github.io/absolutelib/absolutelib.json`
- Maven root: `https://team4308.github.io/absolutelib`
- Artifact: `https://team4308.github.io/absolutelib/ca/team4308/absolutelib-java/1.0.X/absolutelib-java-1.0.X.pom`

If you want to regenerate the Maven repo locally for debugging, you can run:

```powershell
scripts\build-site.bat
```

This runs `gradlew -PpagesPublish=true publishPagesRepo` and writes to `build/pages-maven` and `./site`.

### 3. Tagging and vendor JSON auto-update

The `.github/workflows/update-vendor-json.yml` workflow runs when you push a tag `v*`:

- Extracts the version from the tag name (e.g. `v1.0.5` → `1.0.5`).
- Updates the `"version"` field in `absolutelib.json`.
- Updates the `jsonUrl` to point at the tagged version on raw.githubusercontent.com (for archival).
- Commits and pushes the updated `absolutelib.json`.

For the GitHub Pages path (the URL you give to teams), we keep:

```json
"jsonUrl": "https://team4308.github.io/absolutelib/absolutelib.json"
```

and the `pages.yml` workflow ensures that is updated on each deploy.

### 4. Robot project pattern (examples: `tut`, `tut2`)

In a robot project (like `tut` or `tut2`), the minimal pattern is:

```groovy
// Detect vendordep
def hasAbsoluteLibVendordep = file("vendordeps/absolutelib.json").exists()

dependencies {
    // WPILib
    annotationProcessor wpi.java.deps.wpilibAnnotations()
    implementation wpi.java.deps.wpilib()
    implementation wpi.java.vendor.java()

    // Only use absolutelib if vendordep is installed
    if (hasAbsoluteLibVendordep) {
        implementation "ca.team4308:absolutelib-java:1.0.5"
    }
}

repositories {
    mavenCentral()
    maven { url "https://jitpack.io" } // optional extra if using JitPack directly
}
```

This makes absolutelib optional: projects without the vendordep still build, ones with the vendordep get the dependency.

---

## Features

- Subsystem base with logging and telemetry (`AbsoluteSubsystem`)
- PathPlanner integration with on-the-fly path generation (2025 API)
- Motor and encoder wrappers (CTRE, REV, duty-cycle, CANCoder)
- Simulation-ready patterns
- WPILib chooser helpers for runtime path selection
- Pre-made subsystems (elevator, pivot, etc.)

(See the examples below and generated Javadoc for more details.)

---

## Subsystem Usage Examples

### Pivot Subsystem (Arm/Wrist)

A minimal Pivot subsystem using AbsoluteLib:

```java
import ca.team4308.absolutelib.subsystems.Pivot;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;

public class ArmSubsystem {
    private final Pivot pivot;

    public ArmSubsystem() {
        MotorWrapper leader = MotorWrapper.createTalonFX(10);
        EncoderWrapper encoder = EncoderWrapper.createCANCoder(20);

        Pivot.Config config = new Pivot.Config()
            .withLeader(leader)
            .withEncoder(encoder)
            .pid(0.5, 0.0, 0.1)
            .ff(0.1, 0.5, 0.0, 0.0)
            .gear(100.0)
            .limits(-90, 90)
            .tolerance(2.0)
            .inverted(false);

        pivot = new Pivot(config);
        pivot.initialize();
    }

    public void periodic() {
        pivot.periodic();
        // Example: log current angle
        double angleDeg = pivot.getAngleDeg();
        boolean atTarget = pivot.atTarget();
        // Use your preferred logging/telemetry system here
    }

    public void setAngle(double degrees) {
        pivot.setTargetAngleDeg(degrees);
    }

    public void stop() {
        pivot.disable();
    }
}
```

#### Simulation Integration

AbsoluteLib supports WPILib simulation. To enable simulation for the Pivot:

```java
import ca.team4308.absolutelib.subsystems.simulation.PivotSimulation;

Pivot.Config config = new Pivot.Config()
    .withLeader(leader)
    .withEncoder(encoder)
    .withSimulation(new PivotSimulation.Config()
        .gearbox(DCMotor.getNEO(1), 1)
        .gearRatio(100.0)
        .armLength(0.5)
        .armMass(5.0)
        .limits(Math.toRadians(-90), Math.toRadians(90))
        .startAngle(Math.toRadians(0))
        .gravity(true)
    )
    .enableSimulation(true);

pivot = new Pivot(config);
```

When running in simulation (`RobotBase.isSimulation()`), the Pivot will use the physics model for realistic behavior. You can set voltages, read angles, and visualize the simulated arm.

---

### Elevator Subsystem

A minimal Elevator subsystem using AbsoluteLib:

```java
import ca.team4308.absolutelib.subsystems.Elevator;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;

public class ElevatorSubsystem {
    private final Elevator elevator;

    public ElevatorSubsystem() {
        MotorWrapper leader = MotorWrapper.createSparkMax(5);
        EncoderWrapper encoder = leader.getIntegratedEncoder();

        Elevator.Config config = new Elevator.Config()
            .withLeader(leader)
            .withEncoder(encoder)
            .pid(5.0, 0.0, 0.5)
            .ff(0.05, 0.8, 0.0, 0.0)
            .sprocketRadiusMeters(0.025)
            .gear(12.0)
            .limits(0.0, 1.5)
            .tolerance(0.02);

        elevator = new Elevator(config);
        elevator.initialize();
    }

    public void periodic() {
        elevator.periodic();
        double height = elevator.getHeightMeters();
        boolean atTarget = elevator.atTarget();
        // Use your preferred logging/telemetry system here
    }

    public void setHeight(double meters) {
        elevator.setTargetHeightMeters(meters);
    }

    public void stop() {
        elevator.disable();
    }
}
```

#### Simulation Integration

Elevator simulation is similar to Pivot. Provide a simulation config and enable simulation:

```java
import ca.team4308.absolutelib.subsystems.simulation.ElevatorSimulation;

Elevator.Config config = new Elevator.Config()
    .withLeader(leader)
    .withEncoder(encoder)
    .withSimulation(new ElevatorSimulation.Config()
        .motorType(DCMotor.getNEO(2))
        .gearRatio(12.0)
        .massKg(8.0)
        .heightLimits(0.0, 1.5)
        .startHeight(0.0)
    )
    .enableSimulation(true);

elevator = new Elevator(config);
```

---

## Simulation Support

AbsoluteLib subsystems support WPILib simulation out of the box. To use simulation:

- Provide a simulation config (`PivotSimulation.Config`, `ElevatorSimulation.Config`, etc.) when constructing your subsystem.
- Set `.enableSimulation(true)` in the config.
- When running in simulation, the subsystem will use the physics model for realistic feedback.
- You can interact with the simulated subsystem using the same API as real hardware (set voltages, read positions, etc.).
- Telemetry and visualization (e.g., AdvantageScope) are supported.

Simulation is useful for testing control logic, tuning, and visualization before deploying to a real robot.

---

## Path Following (PathPlanner 2025)

AbsoluteLib provides helpers for on-the-fly path generation using PathPlanner:

```java
import ca.team4308.absolutelib.path.OnTheFlyPathing;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// Define constraints
PathConstraints constraints = OnTheFlyPathing.constraints(3.5, 3.0);

// Create a direct path between two poses
Pose2d start = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));
Pose2d end   = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(180));

PathPlannerPath path = OnTheFlyPathing.direct(
    start,
    end,
    constraints,
    Rotation2d.fromDegrees(180),
    0.0,        // end velocity m/s
    true        // prevent alliance flipping
);
```

You can also generate arc and S-curve paths, and select between variants at runtime.

---

## See Also

- [Javadoc](https://team4308.github.io/absolutelib/javadoc/)
- [WPILib Simulation Docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation.html)

---

## Appendix: Quick Reference

### Common Patterns

**Install via WPILib vendordep:**
```text
https://team4308.github.io/absolutelib/absolutelib.json
```

**Gradle dependency:**
```gradle
implementation "ca.team4308:absolutelib-java:1.0.5"
```

**Add Maven repo:**
```gradle
repositories {
    mavenCentral()
    maven { url = uri("https://team4308.github.io/absolutelib") }
}
```

---

### Subsystem Construction Patterns

#### Pivot (Arm/Wrist)
```java
Pivot.Config config = new Pivot.Config()
    .withLeader(MotorWrapper.createTalonFX(10))
    .withEncoder(EncoderWrapper.createCANCoder(20))
    .pid(0.5, 0.0, 0.1)
    .ff(0.1, 0.5, 0.0, 0.0)
    .gear(100.0)
    .limits(-90, 90)
    .tolerance(2.0)
    .inverted(false);

Pivot pivot = new Pivot(config);
pivot.initialize();
```
**Enable simulation:**
```java
config.withSimulation(new PivotSimulation.Config()
    .gearbox(DCMotor.getNEO(1), 1)
    .gearRatio(100.0)
    .armLength(0.5)
    .armMass(5.0)
    .limits(Math.toRadians(-90), Math.toRadians(90))
    .startAngle(Math.toRadians(0))
    .gravity(true)
).enableSimulation(true);
```

#### Elevator
```java
Elevator.Config config = new Elevator.Config()
    .withLeader(MotorWrapper.createSparkMax(5))
    .withEncoder(MotorWrapper.createSparkMax(5).getIntegratedEncoder())
    .pid(5.0, 0.0, 0.5)
    .ff(0.05, 0.8, 0.0, 0.0)
    .sprocketRadiusMeters(0.025)
    .gear(12.0)
    .limits(0.0, 1.5)
    .tolerance(0.02);

Elevator elevator = new Elevator(config);
elevator.initialize();
```
**Enable simulation:**
```java
config.withSimulation(new ElevatorSimulation.Config()
    .motorType(DCMotor.getNEO(2))
    .gearRatio(12.0)
    .massKg(8.0)
    .heightLimits(0.0, 1.5)
    .startHeight(0.0)
).enableSimulation(true);
```

#### Intake
```java
Intake.Config config = new Intake.Config()
    .withLeader(MotorWrapper.createSparkMax(7))
    .pid(0.2, 0.0, 0.01)
    .ff(0.05, 0.1, 0.0, 0.0)
    .inverted(false);

Intake intake = new Intake(config);
intake.initialize();
```
**Enable simulation:**
```java
config.withSimulation(new IntakeSimulation.Config()
    .motorType(DCMotor.getNEO(1))
    .gearRatio(1.0)
    .massKg(1.0)
).enableSimulation(true);
```

#### Shooter
```java
Shooter.Config config = new Shooter.Config()
    .withLeader(MotorWrapper.createTalonFX(15))
    .pid(0.3, 0.0, 0.02)
    .ff(0.1, 0.2, 0.0, 0.0)
    .inverted(false);

Shooter shooter = new Shooter(config);
shooter.initialize();
```
**Enable simulation:**
```java
config.withSimulation(new ShooterSimulation.Config()
    .motorType(DCMotor.getFalcon500(1))
    .gearRatio(1.0)
    .flywheelRadius(0.075)
    .flywheelMass(2.0)
).enableSimulation(true);
```

#### Drivetrain (Differential)
```java
Drivetrain.Config config = new Drivetrain.Config()
    .withLeftLeader(MotorWrapper.createSparkMax(1))
    .withRightLeader(MotorWrapper.createSparkMax(2))
    .trackWidthMeters(0.6)
    .wheelDiameterMeters(0.1524)
    .gearRatio(10.71)
    .pid(0.8, 0.0, 0.1)
    .ff(0.2, 0.5, 0.0, 0.0);

Drivetrain drivetrain = new Drivetrain(config);
drivetrain.initialize();
```
**Enable simulation:**
```java
config.withSimulation(new DrivetrainSimulation.Config()
    .motorType(DCMotor.getNEO(2))
    .trackWidthMeters(0.6)
    .wheelDiameterMeters(0.1524)
    .gearRatio(10.71)
    .robotMassKg(50.0)
).enableSimulation(true);
```

---

### PathPlanner direct path
```java
PathPlannerPath path = OnTheFlyPathing.direct(
    startPose, endPose, constraints, endHeading, 0.0, true
);
```


