# AbsoluteLib V2

[![](https://jitpack.io/v/team4308/absolutelib.svg)](https://jitpack.io/#team4308/absolutelib)
![Java CI](https://github.com/team4308/absolutelib/actions/workflows/gradle.yml/badge.svg)
![License](https://img.shields.io/github/license/team4308/absolutelib)

AbsoluteLib is an FRC utility library for Team 4308 providing reusable subsystems, math helpers, and adapters around WPILib and common vendor APIs. It tryig to unify hardware interaction and simplifying simulation.

---

## Installation

### WPILib Vendor Library (Recommended)
1. Open your robot project in VS Code with the WPILib extension.
2. Press <kbd>Ctrl+Shift+P</kbd> → **WPILib: Manage Vendor Libraries**.
3. Choose **Install new library (online)**.
4. When prompted for a URL, enter:
   ```text
   https://team4308.github.io/absolutelib/lib/absolutelib.json
   ```
5. Save and let Gradle refresh.

### Dependencies
AbsoluteLib automatically pulls in dependencies for:
*   **CTRE Phoenix 5 & 6**
*   **REVLib**
*   **ReduxLib**
*   **PhotonLib**
*   **YAGSL**
*   **AdvantageKit**
*   **PathPlannerLib**
*   **NavX**

---

## Features
- **Unified Motor Wrapper**: Control TalonFX, TalonSRX, VictorSPX, and SparkMax with a single API.
- **Pre-made Subsystems**: `Pivot`, `Elevator`, and `Arm` classes with built-in PID, Feedforward, and Physics Simulation.
- **Vision Integration**: Plug-and-play PhotonVision wrapper supporting multi-camera pose estimation.
- **LED Controller**: Advanced patterns like Larson Scanner, Fire, Rainbow, and Strobe.
- **Simulation First**: All subsystems are designed to work in simulation out of the box.

---

## Usage Examples

### 1. Pivot Subsystem (Arm/Wrist)
The `Pivot` class handles PID control, feedforward (gravity), and simulation automatically.

```java
import ca.team4308.absolutelib.subsystems.Pivot;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.subsystems.simulation.PivotSimulation;
import edu.wpi.first.math.system.plant.DCMotor;

public class Wrist extends Pivot {
    public Wrist() {
        super(new Config()
            // Hardware Setup
            .withLeader(new MotorWrapper(MotorWrapper.MotorType.TALONFX, 15))
            .withEncoder(EncoderWrapper.canCoder(20, 0.0)) // ID, Offset
            
            // Physics & Limits
            .gear(50.0) // 50:1 reduction
            .limits(-90, 90) // Degrees
            .tolerance(1.0)
            
            // Control
            .pid(0.02, 0.0, 0.0)
            .ff(0.0, 0.15, 0.0, 0.0) // kS, kG, kV, kA
            .useSmartMotion(true) // Use on-board motion profiling
            .motion(360, 720) // Max Vel (deg/s), Max Accel (deg/s^2)
            
            // Simulation
            .enableSimulation(true)
            .withSimulation(new PivotSimulation.Config()
                .gearbox(DCMotor.getFalcon500(1), 1)
                .armLength(0.3) // Meters
                .armMass(2.0)   // Kg
            )
        );
    }

    public void goToScore() {
        setTargetAngleDeg(45.0);
    }
}
```

### 2. Unified Motor Wrapper
Control any motor vendor with the same code. Supports "Smart Motion" (Motion Magic / MAXMotion) transparently.

```java
import ca.team4308.absolutelib.wrapper.MotorWrapper;

// Create motors
MotorWrapper shooter = new MotorWrapper(MotorWrapper.MotorType.TALONFX, 10);
MotorWrapper intake = MotorWrapper.sparkMax(11, SparkMax.MotorType.kBrushless);

// Configure
shooter.applyMotorConfig(MotorWrapper.UnifiedMotorConfig.builder()
    .inverted(true)
    .brakeMode(false)
    .currentLimit(40)
    .kP(0.1)
    .kF(0.05)
    .build());

// Control
shooter.setVoltage(12.0);       // Voltage control
intake.set(0.5);                // Percent output
shooter.setSmartPosition(10.0); // Go to 10 rotations using Motion Magic
```

### 3. Vision System
Integrate PhotonVision with multi-camera support and pose estimation.

```java
import ca.team4308.absolutelib.vision.Vision;
import edu.wpi.first.apriltag.AprilTagFields;

public class RobotContainer {
    private final Vision vision;

    public RobotContainer() {
        var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        
        // Define a camera
        var frontCam = new Vision.VisionCamera(
            "FrontCam", 
            layout,
            new Rotation3d(0, 0, 0),           // Camera Rotation
            new Translation3d(0.3, 0, 0.2),    // Camera Position relative to robot center
            VecBuilder.fill(0.5, 0.5, 0.5),    // Single Tag StdDevs
            VecBuilder.fill(0.1, 0.1, 0.1)     // Multi Tag StdDevs
        );

        vision = new Vision(
            drive::getPose, 
            drive.field, 
            layout, 
            frontCam
        );
    }

    public void periodic() {
        // Updates SwerveDrive pose estimator with vision measurements
        vision.updatePoseEstimation(drive);
    }
}
```

### 4. Advanced LED Patterns
Easily apply complex patterns to addressable LEDs.

```java
import ca.team4308.absolutelib.leds.Leds;
import ca.team4308.absolutelib.leds.Patterns;
import edu.wpi.first.wpilibj.util.Color;

public class LedSubsystem {
    private final Leds leds = new Leds(9, 60); // PWM Port 9, 60 LEDs

    public void setIdle() {
        leds.setPattern(Patterns.larsonScanner(Color.kRed, 1.0, 5)); // Cylon effect
    }

    public void setShooting() {
        leds.setPattern(Patterns.strobe(Color.kGreen, 0.1));
    }

    public void setClimbing() {
        leds.setPattern(Patterns.fire(0.3, 0.05)); // Fire effect
    }
    
    public void setDisabled() {
        leds.setPattern(Patterns.rainbowChase());
    }

    public void periodic() {
        leds.periodic();
    }
}
```

---

## Contributing
Feel free to open issues or submit pull requests. Follow the standard GitHub workflow and ensure all tests pass before merging.

---

## License
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.
