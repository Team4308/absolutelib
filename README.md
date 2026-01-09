# AbsoluteLib

AbsoluteLib is an FRC utility library for Team 4308. It provides reusable subsystems + wrappers with a focus on “same code in real + sim”.

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

---

## What you get

### Wrappers
- `MotorWrapper`: TalonFX (Phoenix6), TalonSRX/VictorSPX (Phoenix5), SparkMax (REV) unified API.
- `EncoderWrapper`: unified encoder access (ex: CANCoder, etc).

### Subsystems
- `Pivot`: arm/wrist joint (PID + FF, optional “Smart Motion” path)
- `Elevator`: linear elevator control (PID + FF, optional “Smart Motion” path)

### Simulation
- `PivotSimulation`: physics sim via `SingleJointedArmSim`
- `ElevatorSimulation`: physics sim via `ElevatorSim`

### Vision / LEDs
- `Vision`: PhotonVision wrapper + multi-camera support for pose estimation
- `leds/*`: Addressable LED patterns and simulation helpers

### Example Code
  - Under the folder ./example/example-2026 you can find full robot code for all subsystems + simulation.


---

# Full Simulation: How it works

AbsoluteLib simulations are designed around this pattern:

1. **Subsystem computes output** (PID + FF) → produces a voltage or percent output.
2. Subsystem **applies output to motor** (real hardware OR sim state).
3. In SIM: subsystem also passes the *same computed voltage* into the simulation object.
4. Simulation updates position/velocity and writes them back into:
   - encoder sim position
   - motor controller sim state (when supported)

That is why in SIM you typically call:
- `simulation.setVoltage(lastAppliedVoltage)` (or equivalent)
- `simulation.periodic()` (or `simUpdate(dt)` depending on implementation)

---

# Example Robot Setup (Structure)

Typical places to update simulation:
- `Subsystem.periodic()` (already done in Pivot)
- or `Robot.simulationPeriodic()` if you want centralized sim ticking

**If you centralize**: call `subsystem.periodic()` like normal AND call simulation tick methods there.

---

# MotorWrapper (standalone)

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




# Pivot Subsystem Example Code

## Pivot config + subsystem
```java
import ca.team4308.absolutelib.subsystems.Pivot;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.subsystems.simulation.PivotSimulation;
import edu.wpi.first.math.system.plant.DCMotor;

public class Wrist extends Pivot {
  public Wrist() {
    super(new Pivot.Config()
      .withLeader(new MotorWrapper(MotorWrapper.MotorType.TALONFX, 15))
      .withEncoder(EncoderWrapper.canCoder(20, 0.0)) // CAN id + offset (your wrapper API)

      .gear(50.0)
      .limits(-90, 90)
      .tolerance(1.0)

      // PID + FF
      .pid(0.02, 0.0, 0.0)
      .ff(0.0, 0.15, 0.0, 0.0)

      // Simulation
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

## What makes it “full sim”
- In `Pivot.periodic()` AbsoluteLib already does:
  - compute PID+FF voltage
  - `leader.setVoltage(volts)`
  - in SIM: `simulation.setVoltage(lastAppliedVoltage)` then `simulation.periodic()`
- `PivotSimulation` writes the arm position back into encoder sim and motor sim state.

---

# Elevator Subsystem (FULL working example + SIM)

## Elevator config + subsystem
This example assumes you create in your robot project a minimal `ElevatorSubsystem` wrapper that owns both the `Elevator` and the `ElevatorSimulation`.

```java
import ca.team4308.absolutelib.subsystems.Elevator;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.subsystems.simulation.ElevatorSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

public class ElevatorSubsystem {
  private final MotorWrapper leader = new MotorWrapper(MotorWrapper.MotorType.TALONFX, 31);

  private final EncoderWrapper encoder =
      EncoderWrapper.canCoder(32, /*gearRatio*/ 10.0, /*cpr*/ Elevator.DEFAULT_CANCODER_CPR, /*drumDiameter*/ 0.04);

  private final Elevator elevator;

  private final ElevatorSimulation sim; // created only if sim enabled

  public ElevatorSubsystem() {
    var cfg = Elevator.ElevatorConfig.builder()
        .minHeightMeters(0.0)
        .maxHeightMeters(1.2)
        .gearRatio(10.0)
        .drumDiameterMeters(0.04)
        .maxVelocityMetersPerSec(1.0)
        .maxAccelerationMetersPerSecSq(2.0)
        .toleranceMeters(0.02)
        .enableSimulation(true)
        .build();

    elevator = new Elevator(leader, null, encoder, cfg);

    if (RobotBase.isSimulation() && cfg.enableSimulation) {
      ElevatorSimulation.ElevatorSimulationConfig simCfg = new ElevatorSimulation.ElevatorSimulationConfig();
      simCfg.leader = DCMotor.getFalcon500(1);
      simCfg.gearing = cfg.gearRatio;
      simCfg.carriageMassKg = 8.0;
      simCfg.minHeightMeters = cfg.minHeightMeters;
      simCfg.maxHeightMeters = cfg.maxHeightMeters;
      simCfg.drumRadiusMeters = cfg.drumDiameterMeters / 2.0;
      simCfg.simulateGravity = true;
      simCfg.startHeightMeters = 0.0;

      sim = new ElevatorSimulation("Elevator", simCfg, elevator);
      sim.initialize();
    } else {
      sim = null;
    }
  }

  public void setHeight(double meters) {
    elevator.setPosition(meters);
  }

  public void periodic() {
    elevator.computeOutputPercent(); // main control loop

    // Simulation tick: IMPORTANT
    if (sim != null) {
      // If you change ElevatorSimulation later to use subsystem-computed volts,
      // mirror Pivot’s pattern: sim.setInputVoltage(calculatedVolts).
      sim.simUpdate(0.02);
    }
  }
}
```

### Note about Elevator sim voltage source
Right now your `ElevatorSimulation` reads:
```java
appliedVoltage = realElevator.getLeaderMotor().getAppliedVoltage();
```
If you want the same behavior as Pivot (use “PID+FF total voltage”), you should adapt Elevator to track and expose the computed voltage and feed that into sim (same flow Pivot uses).

---

# Vision (PhotonVision + pose update)

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

# LEDs (patterns)

```java
import ca.team4308.absolutelib.leds.Patterns;
import ca.team4308.absolutelib.leds.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LedLogic {
  private LEDPattern pattern = Patterns.idle();

  public void setError() { pattern = Patterns.error(); }
  public void setFire() { pattern = Patterns.fire(0.3, 0.05); }
  public void setScanner() { pattern = Patterns.larsonScanner(Color.kRed, 1.3, 6); }

  public LEDPattern getPattern() { return pattern; }
}
```

(Your `Leds` class should call `pattern.applyTo(view)` and push the buffer each loop.)

---

## Notes / gotchas (based on current code)

- `Pivot.setTargetAngleDeg()` calls `setPosition(deg)` but does not schedule the returned command. That’s fine if you also drive `targetAngleRad` directly (you do), but don’t expect the Command to run unless you schedule it.
- “Smart Motion”:
  - TalonFX (Phoenix6) supports Motion Magic.
  - TalonSRX/VictorSPX (Phoenix5) can do Motion Magic, but your code/comments may conflict. Document + test per controller.
- Simulation voltage source:
  - Pivot sim uses computed voltage (good).
  - Elevator sim currently reads motor applied voltage; consider mirroring Pivot.

---

## License
MIT (see `LICENSE`).
