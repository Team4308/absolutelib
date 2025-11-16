# AbsoluteLib

[![](https://jitpack.io/v/team4308/absolutelib.svg)](https://jitpack.io/#team4308/absolutelib)

AbsoluteLib is an FRC utility library for Team 4308 providing reusable subsystems, math helpers, and adapters around WPILib and common vendor APIs.

Version: 2.0.0  
Updated: Nov 12, 2025

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
  "https://team4308.github.io/absolutelib/releases",
  "https://jitpack.io"
]
```

The self-hosted Maven repo (under GitHub Pages) exposes:

- Group ID: `ca.team4308`
- Artifact ID: `absolutelib-java`
- Version: `1.0.5`

The path on Pages is:

```text
https://team4308.github.io/absolutelib/releases/ca/team4308/absolutelib-java/1.0.5/absolutelib-java-1.0.5.jar
```

If you want to consume this directly without the vendordep, you can add:

```gradle
repositories {
    mavenCentral()
    maven {
        url = uri("https://team4308.github.io/absolutelib/releases")
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

This is handled by the `pages.yml` workflow:

- It runs on pushes to `main` and tags `v*` (deploy gated to `main`).
- It calls:

  ```bash
  ./gradlew -Pversion="${PUBLISH_VERSION}" -PpagesPublish publishPagesRepo
  ```

- `publishPagesRepo`:
  - Publishes the `absolutelib-java` artifact into `build/pages-maven` using the `GitHubPagesLocal` Maven repository.
  - Generates a minimal HTML index.

- The workflow then copies `build/pages-maven` to `deploy/releases` and deploys it via `actions/deploy-pages`.

Result: `https://team4308.github.io/absolutelib/releases` is a standard Maven repo the vendordep and robot projects can use without credentials.

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

