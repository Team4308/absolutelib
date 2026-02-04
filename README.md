# AbsoluteLib

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
- Phoenix6-Replay (6 and 5)
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
- `TrajectorySolver`: Complete trajectory solving for turret shooters
- `FlywheelGenerator`: Automated flywheel configuration generation and optimization
- `FlywheelSimulator`: Physics-based flywheel and ball exit velocity simulation
- `ProjectileMotion`: Projectile physics with air resistance and spin
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

In simulation, you typically call:
- `simulation.setVoltage(lastAppliedVoltage)` (or equivalent)
- `simulation.periodic()` (or `simUpdate(dt)` depending on implementation)

---

## Subsystem Examples

### MotorWrapper (Standalone)

```java
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import com.revrobotics.spark.SparkMax;

public class ShooterIO {
    private final MotorWrapper leader = new MotorWrapper(MotorWrapper.MotorType.TALONFX, 10);
    private final MotorWrapper feeder = MotorWrapper.sparkMax(11, SparkMax.MotorType.kBrushless);

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

### Pivot Subsystem

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
            .withEncoder(EncoderWrapper.canCoder(20, 1.0))
            .gear(50.0)
            .limits(-90, 90)
            .tolerance(1.0)
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
                .gravity(true)
            )
        );
    }

    public void goToStow() { setTargetAngleDeg(0.0); }
    public void goToScore() { setTargetAngleDeg(45.0); }
}
```

In `Pivot.periodic()`, AbsoluteLib:
- Computes PID+FF voltage
- Calls `leader.setVoltage(volts)`
- In SIM: `simulation.setVoltage(lastAppliedVoltage)` then `simulation.periodic()`

---

### Elevator Subsystem

Linear elevator with profiled PID control.

```java
import ca.team4308.absolutelib.subsystems.Elevator;
import ca.team4308.absolutelib.subsystems.simulation.ElevatorSimulation;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorSubsystem {
    private final Elevator elevator;

    public ElevatorSubsystem() {
        MotorWrapper leader = new MotorWrapper(MotorWrapper.MotorType.SPARKMAX, 10);
        EncoderWrapper encoder = EncoderWrapper.ofMechanismRotations(
            leader::getPosition,
            (val) -> leader.asSparkMax().getEncoder().setPosition(val),
            0.05 // drum diameter meters
        );

        Elevator.Config config = new Elevator.Config()
            .withLeader(leader)
            .withEncoder(encoder)
            .gear(10.0)
            .drumRadius(0.02)
            .limits(0.0, 1.2)
            .tolerance(0.02)
            .pid(5.0, 0.0, 0.1)
            .ff(0.0, 0.5, 0.0, 0.0)
            .motion(1.0, 2.0);

        // Simulation config
        ElevatorSimulation.ElevatorSimulationConfig simConfig = 
            new ElevatorSimulation.ElevatorSimulationConfig();
        simConfig.leader = DCMotor.getNEO(2);
        simConfig.gearing = 10.0;
        simConfig.carriageMassKg = 5.0;
        simConfig.drumRadiusMeters = 0.02;
        simConfig.minHeightMeters = 0.0;
        simConfig.maxHeightMeters = 1.2;
        simConfig.simulateGravity = true;

        config.withSimulation(simConfig);
        
        elevator = new Elevator(config);
        elevator.initialize();
    }

    public Command moveToHeight(double meters) {
        return elevator.setPosition(meters);
    }
}
```

---

### Arm Subsystem (Multi-Joint with IK)

Multi-DOF arm with inverse kinematics for end-effector positioning.

```java
import ca.team4308.absolutelib.subsystems.Arm;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;

public class ExampleArm {
    private final Arm arm;
    private final Arm.Joint shoulder;
    private final Arm.Joint elbow;

    public ExampleArm() {
        arm = new Arm();

        // Configure shoulder joint
        MotorWrapper shoulderMotor = new MotorWrapper(MotorWrapper.MotorType.SPARKMAX, 30);
        EncoderWrapper shoulderEncoder = EncoderWrapper.ofMechanismRotations(
            shoulderMotor::getPosition, 
            (val) -> shoulderMotor.asSparkMax().getEncoder().setPosition(val),
            1.0 / Math.PI
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

        // Configure elbow joint (similar pattern)
        // ...

        arm.enableSimulation(true);
        arm.initialize();
    }

    // Move to XY position using Inverse Kinematics
    public Command moveToPoint(double x, double y) {
        return Commands.runOnce(() -> arm.setGoalPose(x, y));
    }

    // Move joints to specific angles
    public Command moveToAngles(double shoulderDeg, double elbowDeg) {
        return Commands.runOnce(() -> 
            arm.setTargetAngles(Math.toRadians(shoulderDeg), Math.toRadians(elbowDeg)));
    }
}
```

---

### EndEffector Subsystem

Base class for intakes, claws, and manipulators.

```java
import ca.team4308.absolutelib.subsystems.EndEffector;
import ca.team4308.absolutelib.wrapper.MotorWrapper;

public class Intake {
    private final EndEffector endEffector;

    public Intake() {
        MotorWrapper motor = new MotorWrapper(MotorWrapper.MotorType.SPARKMAX, 50);
        
        EndEffector.Config config = new EndEffector.Config()
            .withLeader(motor)
            .inverted(false);

        endEffector = new EndEffector(config);
        endEffector.initialize();
    }

    public void intake() { endEffector.start(0.8); }
    public void outtake() { endEffector.start(-0.6); }
    public void stop() { endEffector.stop(); }
}
```

---

### Vision (PhotonVision + Pose Update)

```java
import ca.team4308.absolutelib.vision.Vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionSubsystem {
    private final Vision vision;

    public VisionSubsystem(SwerveDrive drive) {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        var frontCam = new Vision.VisionCamera(
            "FrontCam",
            layout,
            new Rotation3d(0, 0, 0),
            new Translation3d(0.30, 0.0, 0.20),
            VecBuilder.fill(0.8, 0.8, 2.0),
            VecBuilder.fill(0.3, 0.3, 1.0)
        );

        vision = new Vision(drive::getPose, drive.field, layout, frontCam);
    }

    public void periodic(SwerveDrive drive) {
        vision.updatePoseEstimation(drive);
        vision.updateVisionField();
    }
}
```

---

### LEDs (Patterns)

```java
import ca.team4308.absolutelib.leds.Patterns;
import ca.team4308.absolutelib.leds.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LedLogic {
    private LEDPattern pattern = Patterns.idle();

    public void setError() { pattern = Patterns.error(); }
    public void setSuccess() { pattern = Patterns.success(); }
    public void setRainbow() { pattern = Patterns.rainbowChase(); }
    public void setAlliance() { pattern = Patterns.getAlliancePattern(); }
    public void setProgress(double progress) { 
        pattern = Patterns.createProgressPattern(Color.kGreen, progress); 
    }

    public LEDPattern getPattern() { return pattern; }
}
```

---

## Trajectory System (2026 REBUILT)

### Quick Start

```java
import ca.team4308.absolutelib.math.trajectories.*;

// Create solver for 2026 game
TrajectorySolver solver = TrajectorySolver.forGame2026();

// Define shot parameters
ShotInput input = ShotInput.builder()
    .shooterPositionMeters(1.0, 2.0, 0.5)
    .targetPositionMeters(5.0, 5.0, 2.5)
    .shotPreference(ShotInput.ShotPreference.FASTEST)
    .build();

// Get best shot angle
double pitchDegrees = solver.solveBestPitchDegrees(input);
```

### Multi-Candidate Shot Selection

```java
// Find all possible shots sorted by confidence
ShotCandidateList candidates = solver.findAllCandidates(input);

// Get the fastest shot
Optional<ShotCandidate> fastest = candidates.getFastest();
if (fastest.isPresent()) {
    ShotCandidate shot = fastest.get();
    System.out.println("Pitch: " + shot.getPitchAngleDegrees() + "°");
    System.out.println("Velocity: " + shot.getRequiredVelocityMps() + " m/s");
    System.out.println("Time of Flight: " + shot.getTimeOfFlightSeconds() + "s");
}

// Or get shots by preference
candidates.getMostAccurate();    // Most accurate shot
candidates.getMostStable();      // Most stable angle
candidates.getMaxClearance();    // Highest arc for obstacles
candidates.getBestHighArc();     // Best high-arc shot
candidates.getBestLowArc();      // Best low-arc shot
```

### Legacy API

```java
TrajectoryResult result = solver.solve(input);

if (result.isSuccess()) {
    System.out.println("Pitch: " + result.getPitchAngleDegrees() + "°");
    System.out.println("RPM: " + result.getRecommendedRpm());
    System.out.println("Flywheel: " + result.getFlywheelConfig().getName());
    System.out.println("Confidence: " + result.getConfidence() + "%");
}
```

### Flywheel Configuration

```java
import ca.team4308.absolutelib.math.trajectories.flywheel.*;
import ca.team4308.absolutelib.math.trajectories.gamepiece.*;

GamePiece ball = GamePieces.REBUILT_2026_BALL;
FlywheelGenerator generator = new FlywheelGenerator(ball);

// Generate and evaluate configurations for target velocity
FlywheelGenerator.GenerationResult result = generator.generateAndEvaluate(15.0);

// Get best configuration
FlywheelConfig best = result.bestConfig.config;
System.out.println("Best: " + best.toString());
```

### Custom Flywheel Configuration

```java
FlywheelConfig custom = FlywheelConfig.builder()
    .name("Custom Shooter")
    .arrangement(FlywheelConfig.WheelArrangement.DUAL_OVER_UNDER)
    .wheelDiameterInches(4.0)
    .wheelWidthInches(2.0)
    .material(WheelMaterial.GREEN_COMPLIANT)
    .compressionRatio(0.12)
    .wheelCount(2)
    .motor(FRCMotors.KRAKEN_X60)
    .motorsPerWheel(1)
    .gearRatio(1.0)
    .build();

// Simulate at specific RPM
FlywheelSimulator simulator = new FlywheelSimulator(custom, ball);
FlywheelSimulator.SimulationResult sim = simulator.simulateAtRpm(5000);

System.out.println("Exit Velocity: " + sim.exitVelocityMps + " m/s");
System.out.println("Ball Spin: " + sim.ballSpinRpm + " RPM");
```

### Game Piece Specifications

```java
// 2026 REBUILT Ball
GamePiece ball = GamePieces.REBUILT_2026_BALL;
// Diameter: 5.91 inches
// Mass: 0.448-0.5 lbs (avg 0.474 lbs)
// Material: High-density foam
// Shape: Sphere

// Other supported game pieces
GamePieces.CRESCENDO_2024_NOTE;           // 2024 Note (Ring)
GamePieces.RAPID_REACT_2022_CARGO;        // 2022 Cargo
GamePieces.INFINITE_RECHARGE_POWER_CELL;  // 2020 Power Cell
GamePieces.DEEP_SPACE_2019_CARGO;         // 2019 Cargo

// Get by year
GamePiece piece = GamePieces.getByYear(2022);
```

### Shot Preferences

```java
ShotInput input = ShotInput.builder()
    .shooterPositionMeters(x, y, z)
    .targetPositionMeters(tx, ty, tz)
    .shotPreference(ShotInput.ShotPreference.FASTEST)      // Minimize time of flight
    // .shotPreference(ShotInput.ShotPreference.MOST_ACCURATE)  // Best accuracy
    // .shotPreference(ShotInput.ShotPreference.MOST_STABLE)    // Most stable angle
    // .shotPreference(ShotInput.ShotPreference.HIGH_CLEARANCE) // Avoid obstacles
    // .shotPreference(ShotInput.ShotPreference.MIN_VELOCITY)   // Minimum velocity
    .build();
```

---

## Notes

- `Pivot.setTargetAngleDeg()` updates the target angle but does not schedule the returned command automatically.
- **Smart Motion**:
  - TalonFX (Phoenix6) supports Motion Magic
  - TalonSRX/VictorSPX (Phoenix5) can use Motion Magic with configuration
  - Victor SPX and TalonSRX-based motors (CIMs, 775) have not been tested
