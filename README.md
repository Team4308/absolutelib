# AbsoluteLib V2

AbsoluteLib is an FRC utility library for Team 4308. It provides reusable subsystems and wrappers with a focus on "same code in real + sim".

- **Vendor install:** `https://team4308.github.io/absolutelib/lib/absolutelib.json`
- **Docs site:** [https://team4308.github.io/absolutelib/](https://team4308.github.io/absolutelib/)
- **JavaDoc site:** [https://team4308.github.io/absolutelib/docs/javadoc](https://team4308.github.io/absolutelib/docs/javadoc)

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

### Math Utilities
- `ChineseRemainderSolver`: Generic CRT solver + turret anti-windup (shortest-path rotation within cable limits)
- `DoubleUtils`: Clamping, normalization, range mapping
- `Vector2` / `Vector3`: Lightweight vector math

### Trajectory System (2026 REBUILT)
- Complete ballistic calculation engine for trajectories with obstacle avoidance.
- Coprocessor offloading and unified state machines via `ShooterSystem`.
- Supports offline precomputing of shot lookup tables via JSON configurations.

---

## Example Code

To keep the documentation clean and ensure you are looking at up-to-date, runnable code, **all example code has been moved to the `/example` directory.**

Check out `./example/example-2026-Imported` (or the folder corresponding to the current year) for full robot code demonstrating all subsystems, trajectory integration, and simulation.

---

## Updating AbsoluteLib in Your Robot Project

When a new version of AbsoluteLib is released, follow these steps to update:

### Option 1: Automatic (Recommended)

1. WPILib VS Code → `Ctrl+Shift+P`
2. `WPILib: Manage Vendor Libraries`
3. `Check for updates (online)`
4. If AbsoluteLib shows an update, accept it.

### Option 2: Manual Re-install

1. WPILib VS Code → `Ctrl+Shift+P`
2. `WPILib: Manage Vendor Libraries`
3. `Install new library (online)`
4. Paste the vendor JSON URL.
5. If prompted to replace the existing version, confirm.
6. Rebuild your project (`Ctrl+Shift+P` → `WPILib: Build Robot Code`).

> **Tip:** After updating, always do a clean build (`./gradlew clean build`) to avoid stale cached artifacts.

---

## Contributing

We welcome contributions to AbsoluteLib! If you are part of Team 4308 or a community member looking to improve the library:

1. **Clone the repository:** `git clone https://github.com/Team4308/absolutelib.git`
2. **Branch out:** Create a feature branch (`git checkout -b feature/your-feature-name`).
3. **Make your changes:** Ensure your code works both in real hardware configurations and in simulation.
4. **Build and Test:** Run `./gradlew build` locally to ensure no compilation errors or checkstyle failures.
5. **Submit a PR:** Open a Pull Request on GitHub with a clear description of your changes.

---

## Releasing a New Version (For Maintainers)

To publish a new release of AbsoluteLib, simply run:

```bat
scripts\release.bat
```

This single script handles the entire release process, including safety checks, version bumping, backups, git commits, Maven artifact generation, and deployment to GitHub Pages.

## Updating Libraries and WPILib Versions

1. **WPILib** — Go into the `build.gradle` file and change `wpilibVersion` to the year. You may also have to update `WPILibRepositoriesPlugin`.
2. **Libraries** — Also in the `build.gradle` file, scroll down to the `dependencies` block and update each package as needed.
