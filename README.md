# AbsoluteLib V2

[![](https://jitpack.io/v/team4308/absolutelib.svg)](https://jitpack.io/#team4308/absolutelib)
![Java CI](https://github.com/team4308/absolutelib/actions/workflows/gradle.yml/badge.svg)
![License](https://img.shields.io/github/license/team4308/absolutelib)




AbsoluteLib is an FRC utility library for Team 4308 providing reusable subsystems, math helpers, and adapters around WPILib and common vendor APIs.

Updated: Nov 21, 2025

* Note for older version of absolutelib (v1) Check Legacy branch and LegacyV2 branch. 


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

## Features

- Subsystem base with logging and telemetry (`AbsoluteSubsystem`)
- PathPlanner integration with on-the-fly path generation (2025 API)
- Motor and encoder wrappers (CTRE, REV, duty-cycle, CANCoder)
- Simulation-ready patterns
- WPILib chooser helpers for runtime path selection
- Pre-made subsystems (elevator, pivot, arm, etc.)
- PID Auto-Tuning Utilities
- Music Player for TalonFX
- LED Pattern Generators

---

## Subsystem Usage Examples

### Pivot Subsystem (Arm/Wrist)

A minimal Pivot subsystem using `Pivot.Config` builder:

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
            .inverted(false)
            // Enable simulation with auto-generated physics
            .enableSimulation(true); 

        pivot = new Pivot(config);
        pivot.initialize();
    }

    public void periodic() {
        pivot.periodic();
        // Example: log current angle
        double angleDeg = pivot.getAngleDeg();
        boolean atTarget = pivot.atTarget();
    }

    public void setAngle(double degrees) {
        pivot.setTargetAngleDeg(degrees);
    }

    public void stop() {
        pivot.disable();
    }
}
```

### Elevator Subsystem

A minimal Elevator subsystem using `Elevator.ElevatorConfig` builder:

```java
import ca.team4308.absolutelib.subsystems.Elevator;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;

public class ElevatorSubsystem extends Elevator {
    
    public ElevatorSubsystem() {
        super(
            MotorWrapper.createSparkMax(5),
            null, // No specific motor config
            EncoderWrapper.createSparkMaxEncoder(5), // Use internal encoder
            Elevator.ElevatorConfig.builder()
                .minHeightMeters(0.0)
                .maxHeightMeters(1.5)
                .gearRatio(12.0)
                .drumDiameterMeters(0.05)
                .maxVelocityMetersPerSec(2.0)
                .maxAccelerationMetersPerSecSq(4.0)
                .toleranceMeters(0.02)
                .build()
        );
        
        // Set PID and FF
        setPositionPID(5.0, 0.0, 0.5);
        setFeedforwardGains(0.05, 0.8, 0.0, 0.0);
        
        initialize();
    }
}
```

### Multi-Joint Arm (with IK)

The `Arm` class supports multiple joints and Inverse Kinematics (IK).

```java
import ca.team4308.absolutelib.subsystems.Arm;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;

public class MyRobotArm extends Arm {
    
    public MyRobotArm() {
        // Joint 1 (Shoulder)
        addJoint(
            MotorWrapper.createTalonFX(1),
            null,
            EncoderWrapper.createCANCoder(11),
            Arm.JointConfig.builder()
                .minAngleRad(Math.toRadians(-90))
                .maxAngleRad(Math.toRadians(90))
                .linkLengthMeters(0.8)
                .build()
        );

        // Joint 2 (Elbow)
        addJoint(
            MotorWrapper.createTalonFX(2),
            null,
            EncoderWrapper.createCANCoder(12),
            Arm.JointConfig.builder()
                .minAngleRad(Math.toRadians(0))
                .maxAngleRad(Math.toRadians(150))
                .linkLengthMeters(0.6)
                .build()
        );
        
        initialize();
    }
    
    public void moveToCoordinate(double x, double y) {
        // Solves IK and moves joints
        setGoalPose(x, y);
    }
}
```

### EndEffector

Extend `EndEffector` for simple mechanisms like intakes or claws.

```java
import ca.team4308.absolutelib.subsystems.EndEffector;
import ca.team4308.absolutelib.wrapper.MotorWrapper;

public class Intake extends EndEffector {
    private final MotorWrapper motor;

    public Intake() {
        motor = MotorWrapper.createSparkMax(3);
        // Configure motor...
    }

    public void run(double speed) {
        motor.set(speed);
    }

    @Override
    public void stop() {
        motor.set(0);
    }
}
```

---

## PID Tuning Utilities

AbsoluteLib includes powerful tools for tuning PID controllers, located in `ca.team4308.absolutelib.pid`.

### SimpleTune (Relay / Bang-Bang Auto-Tuner)

The `SimpleTune` class uses the Åström–Hägglund Relay method to automatically estimate the **Ultimate Gain (Ku)** and **Ultimate Period (Tu)** of your system. These values can then be used to calculate PID constants using standard tuning rules (Ziegler-Nichols, Tyreus-Luyben, etc.).

```java
import ca.team4308.absolutelib.pid.SimpleTune;

// 1. Initialize the tuner
// setpoint: target value (e.g., 90 degrees)
// relayOutput: voltage to apply (e.g., 2.0V)
// hysteresis: noise threshold (e.g., 0.5 degrees)
SimpleTune tuner = new SimpleTune(90.0, 2.0, 0.5);

// 2. In your periodic loop:
public void periodic() {
    double measurement = getMechanismPosition();
    
    // Get the output voltage to apply to the motor
    double output = tuner.update(measurement);
    motor.setVoltage(output);
    
    if (tuner.isFinished()) {
        motor.setVoltage(0);
        
        // 3. Calculate constants when done
        SimpleTune.PIDResult result = tuner.calculateConstants(SimpleTune.TuningRule.ZIEGLER_NICHOLS);
        System.out.println("Calculated PID: " + result);
        // Result contains kP, kI, kD, and estimated Feedforward (kS, kV, kG)
    }
}
```

### PIDOptimizer (Twiddle)

For fine-tuning existing constants, `PIDOptimizer` uses Coordinate Descent (Twiddle) to iteratively improve performance based on a score (like error over time).

```java
import ca.team4308.absolutelib.pid.SimpleTune.PIDOptimizer;

// Initialize with starting constants
PIDOptimizer optimizer = new PIDOptimizer(0.1, 0.0, 0.0);

// In your tuning routine:
// 1. Run a test cycle
// 2. Calculate a score (lower is better)
double score = calculateErrorScore();

// 3. Get next constants to try
SimpleTune.PIDResult nextConstants = optimizer.tune(score);
applyConstants(nextConstants);
```

---

## Music Player

The `musicPlayer` class allows you to play CHRP music files through Falcon 500 (TalonFX) motors.

```java
import ca.team4308.absolutelib.other.musicPlayer;
import com.ctre.phoenix.music.Orchestra;

// Initialize
Orchestra orchestra = new Orchestra();
musicPlayer player = new musicPlayer(orchestra);

// Add motors to the orchestra
player.addInstruments(new MotorWrapper[] { leftLeader, rightLeader });

// Load and play a song
// File path is relative to the deploy directory or absolute
player.loadSong("deploy/song.chrp"); 
player.playSong();
```

---

## Controls & Wrappers

AbsoluteLib provides wrappers to simplify input handling.

- **`JoystickHelper`**: Adds deadband, squaring, and curve mapping to standard joysticks.
- **`XBoxWrapper`**: specific helper for Xbox controllers.
- **`RazerWrapper`**: specific helper for Razer controllers.

```java
import ca.team4308.absolutelib.control.JoystickHelper;

JoystickHelper driver = new JoystickHelper(0);
double speed = driver.getLeftY(); // Automatically applies deadband
```

---

## LED Patterns

Easily control addressable LEDs (WS2812B) with the `Leds` class and `Patterns`.

```java
import ca.team4308.absolutelib.leds.Leds;
import ca.team4308.absolutelib.leds.Patterns;

// Initialize on PWM port 9 with 60 LEDs
Leds leds = new Leds(9, 60);

// Set a pattern
leds.setPattern(Patterns.rainbow());
// or
leds.setPattern(Patterns.solid(Color.kRed));

// Must call in periodic
leds.periodic();
```

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
- [Examples](https://github.com/team4308/absolutelib/tree/main/examples)
- [WPILib Simulation Docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation.html)

---

## Maintainers: How to Update and Publish

Please update the changelog with any notable updates 

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
