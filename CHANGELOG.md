# Changelog for AbsoluteLib v2

## 2.0.0 - Season Ready!
  - Patched any bugs found in simulations and is now ready be used for the season!
  - Removed any warnings 
  - Fixed the website and build errors
  - other minor things fixed lowkey good enough!


## 1.5.3 - Website Redesgin.
  - Remade the docs
  - Updated the readme to show the most up to date example code


## 1.5.2 — Overhaul

- Trajectory solver: the trajectory system will no longer receive breaking major updates; future changes will preserve backwards compatibility.
- Javadoc updates: rewrote many Javadoc comments for clarity.
- Sweep improvements: the `SWEEP` mode now favors lower angles by default for improved results.


## 1.5.1

### Dual-Mode Solver — Constraint + Improved Sweep
- **Both solver strategies now available** via `TrajectorySolver.SolveMode` enum:
  - **`CONSTRAINT`** (default): Two-constraint algebraic solver. Computes exact (pitch, velocity) from target geometry + rim clearance. Fastest and most deterministic.
  - **`SWEEP`**: Improved angle sweep. Tests every pitch from min→max at 0.5° steps, calculates the exact velocity needed per angle, simulates with full RK4 physics, refines with binary-search velocity correction, and picks the angle with the **smallest miss distance**. No scoring, no weights — pure accuracy selection.
- **`CONSTRAINT` mode automatically falls back to `SWEEP`** if the constraint solver fails (e.g., no valid clearance found within +3.0m). The sweep acts as a robust fallback.
- **`SWEEP` mode does NOT fall back** — it runs the sweep only.

### New API
- **`solver.setSolveMode(TrajectorySolver.SolveMode.CONSTRAINT)`** — switch between modes at runtime
- **`solver.getSolveMode()`** — query current mode
- Both modes share the same validation pipeline: collision, clearance, arc height, hit detection, flyover, entry angle, and velocity refinement.

## 1.5.0

### Constraint-Based Solver 
- **Replaced the angle sweep with a two-constraint algebraic solver**. Instead of testing 70+ pitch angles and picking the best, the solver now computes the **single exact (pitch, velocity) pair** from two physical constraints:
  1. Ball passes through target center at `(distance, heightDiff)`
  2. Ball clears the rim edge at `(distance − radius, heightDiff + clearance)`
- This eliminates scoring, searching, and miss-distance comparison entirely. One deterministic solution per solve — no ambiguity.
- **Full physics verification**: The vacuum analytical solution is then simulated with RK4 (air drag, Magnus spin, game piece properties) and refined with 8-iteration binary-search velocity correction. Same physics fidelity, dramatically simpler selection.
- **Obstacle avoidance via clearance escalation**: If the analytical trajectory collides with an obstacle or fails clearance checks, the solver increases `rimClearance` in 0.25m steps (up to +3.0m) until the arc is high enough to clear. No sweep fallback needed.
- **Moving target refinement**: For moving robots, the solver iterates twice with actual time-of-flight to refine the lead prediction — same concept as TurretCalculator's `iterativeMovingShotFromFunnelClearance`.
- **`solveForCurrentRpm()` also updated**: Uses the high-arc angle formula `θ = atan((v² + √(v⁴ − g(gd² + 2hv²))) / (gd))` instead of sweeping. Direct analytical solution for fixed-velocity scenarios.

### New Configuration
- **`SolverConstants.rimClearanceMeters`** (default 0.15m): Height above the target rim the ball must achieve at the rim edge. Controls arc steepness — larger = steeper entry, smaller = flatter. One ball radius (~0.075m) is the physical minimum.
- **`computeConstraintSolution(d, h, r, c)`**: New static method exposing the two-constraint parabolic solver. Returns `[pitchRadians, velocityMps]` for any geometry.

## 1.4.1

### Scoring Removed — Pure Accuracy Selection
- **Removed all scoring**: The solver no longer scores candidates. Instead, it filters valid trajectories (collision-free, on-target, correct entry angle, no flyover) and picks the one **closest to dead center** by miss distance. No points, no weights, no tuning — just the most accurate shot.
- **Selection by miss distance**: Among all valid candidates, the solver selects the trajectory with the smallest horizontal distance from the target center at the rim-crossing plane. A hit 2cm off-center always beats a hit 5cm off-center, regardless of entry angle, height, or flight time.
- **`ScoringWeights` deprecated**: The class and `setScoringWeights()`/`getScoringWeights()` are `@Deprecated`. The solver ignores them entirely.
- **`scoreCandidate()` removed**: The internal scoring method is gone. No replacement — selection is a simple `missDistance < bestMissDistance` comparison.

### Debug Info Updated
- `SolveDebugInfo` now tracks **miss distance** (meters from center) instead of score points.
- New `getBestMissDistance()` and `CandidateInfo.getMissDistance()` methods.
- `getBestScore()` and `getScore()` are `@Deprecated` (return miss distance for compatibility).
- Debug tables and summaries show `miss=X.XXXXm` instead of `score=X.X`.

### Tighter Hit Detection
- **Default tolerance multipliers reduced to 1.0** (`hoopToleranceMultiplier`, `basketDescentToleranceMultiplier`). The ball must land within the actual target radius — no more inflated acceptance zones.
- **Rim-plane crossing always detected**: Simulation stops when the ball descends through targetZ regardless of horizontal distance.
- **Velocity binary-search refinement**: 8-iteration binary search finds the lowest valid velocity for the steepest possible entry angle.

### Other
- Updated `ExampleShooter`: removed all weight configuration, tightened target radius to 0.45m, debug output uses miss distance instead of score.

# 1.3.6 - 1.3.9
- Scoring update
- Re-made the Website

# 1.3.5

- **Debugging!**: New Website part showing greaterlogs of the trajectorys that are generated to help debug and tune it.
- **Path Visuals**: Re-added the 3D Array of path points.

## 1.3.4
- **Hybrid shooter system** (`ca.team4308.absolutelib.math.trajectories.shooter`): New package that combines a lookup table, physics solver, RPM feedback, and movement compensation into a unified shot-calculation pipeline.
- **ShotLookupTable**: Fixed-size sorted lookup table with linear interpolation and edge clamping. Add tested distance/pitch/RPM entries and retrieve interpolated shot parameters in constant time.
- **ShooterConfig**: Immutable builder-pattern configuration for the shooter system. Covers pitch limits, RPM limits, RPM-to-velocity conversion, distance limits, RPM feedback thresholds, movement compensation gain, and safety limits. Includes a `defaults2026()` preset.
- **ShotMode**: Enum with six operating modes — `LOOKUP_ONLY`, `SOLVER_ONLY`, `LOOKUP_WITH_SOLVER_FALLBACK`, `SOLVER_WITH_LOOKUP_FALLBACK`, `BLENDED`, and `MANUAL`.
- **ShooterSystem**: Main orchestrator that runs the full pipeline: mode-based base calculation → movement compensation → RPM feedback correction → safety validation → fallback. Single `calculate()` call per loop.
- **RPMCorrector**: Linear pitch correction when the flywheel is below target speed. Compensates for battery sag, wheel slip, and motor heating without a full trajectory re-solve.
- **MovementCompensator**: Iterative shoot-on-the-fly compensation. Adjusts effective distance and RPM based on chassis velocity so shots land accurately while the robot is moving.
- **SafetyValidator**: Pre-fire safety checks for pitch limits, RPM range, exit velocity, distance, and flywheel readiness. Returns detailed failure reasons.
- **Simplified ExampleShooter**: Rewrote the example from ~1000 lines down to ~280 lines using the new `ShooterSystem` API.

## 1.3.3
- **Per-pitch velocity calculation**: The solver now calculates the exact velocity needed for each pitch angle using projectile kinematics, instead of using a single fixed velocity. This eliminates most flyover rejections and lets the solver find solutions at angles that were previously unreachable.
- **Floating-point pitch loop fix**: Changed pitch iteration from accumulated `+= step` to integer-indexed `min + i * step`, preventing the final angle from being skipped due to IEEE 754 drift. With 300 iterations of `+= 0.1`, the last angle (e.g., 37.5°) was silently excluded.
- **Arc height fallback**: When no candidate passes the `minArcHeightMeters` check but valid trajectories exist (collision-free, on-target, no flyover), the solver now falls back to the best of those. Hardware pitch limits often prevent achieving the required apex height at certain positions — the solver now handles this gracefully instead of returning OUT_OF_RANGE.
- **Angle-dependent drag compensation**: Steep pitch angles now receive less drag compensation since the ball spends less time traveling horizontally. Scales as `1 + (baseDragComp - 1) * cos(pitch)`. Reduces flyovers at high angles (6m test: 27 flyovers → 10) and improves accepted candidate count (14 → 30).
- **Smart force-high-arc**: When hardware pitch limits restrict the range (e.g., max 37.5°), forcing high-arc from 35°+ would leave only 2.5° of range. Now skips the force if the remaining range would be less than 15°, allowing the full hardware range to be searched.

## 1.3.2
- Fix Runtime error

## 1.3.1
- Added `SolveDebugInfo` class that tracks exactly why every candidate angle is accepted or rejected during a solve. Records rejection reason (collision, arc too low, clearance, missed target, flyover), score, closest approach, max height, TOF, and full trajectory for each tested pitch angle.
- Instrumented `TrajectorySolver.solve()` with debug recording at all 5 filter points. Enable with `solver.setDebugEnabled(true)`.
- Added `getDebugInfo()` to `TrajectoryResult` — returns the full `SolveDebugInfo` for the solve that produced the result.
- Added debug telemetry publishing to `ExampleShooter` — publishes per-rejection-type counts, best score/pitch, up to 10 accepted candidate paths, and one sample rejected path per rejection type to NetworkTables.
- Added `trajectory-debug.html` — a standalone NT4 WebSocket dashboard for real-time solver debugging. Shows rejection breakdowns, shot parameters, 2D trajectory visualization, live log, and raw NT value table. Connect to your robot IP on port 5810.

## 1.3.0

- Added flyover detection — the solver now rejects trajectories where the ball passes above the target without descending into it. At the point of closest horizontal approach, the ball must be within one target radius of the target height or the trajectory is skipped.
- Added distance-scaled drag compensation. Instead of applying a flat 1.8x drag multiplier at all distances, velocity compensation now linearly ramps from 1.0 at close range (≤3m) to the full multiplier at 8m+. Fixes medium-range shots (3–5m) being wildly over-powered.
- Applied flyover and drag compensation checks across all solve paths: `solve()`, `solveAtCurrentRpm()`, and `findAllCandidates()`.

## 1.2.9

- Fixed critical bug where `simulate()` parameters were shifted by one position — `ballSpinRpm` was being passed as velocity, causing absurd TOF (5s) and max heights (70m+). This was the root cause of pitch always being ~40 deg and TOF always being 5 seconds.
- Added obstacle-aware trajectory solving. You can now define field obstacles like the 2026 hub with `ObstacleConfig` and the solver will automatically reject trajectories that collide with them.
- Added collision grace distance — balls near the launch point skip collision checks so you can shoot from close to (or inside) an obstacle's footprint.
- Added opening exemption — balls descending into the basket opening aren't blocked by the obstacle structure above it.
- Added `solveAtCurrentRpm()` to `TrajectorySolver` for RPM feedback. When the flywheel hasn't fully spun up, it adjusts pitch angle to compensate for lower velocity.
- Added RPM feedback loop to `ExampleShooter` with `setCurrentRpmSupplier()`, `setRpmFeedbackEnabled()`, and `setRpmFeedbackThreshold()`.
- Reduced `SolverConstants` from 56 fields down to 19. Removed all the scoring weight constants that nobody should be touching.
- Added `isWithinOpening()` to `ObstacleConfig` for checking if a point is within the basket opening.
- Rebalanced scoring — reduced stability weight, increased speed weight so the solver actually picks faster shots.
- Added drag compensation to velocity estimation.

## 1.2.8

- Added `SolverConstants` class with 45 parameters for the trajectory system. These numbers more then likely are fine as they are, but incase a change needs to be made its there
- Removed almost al magic numbers found in the trajectories slover.
- Added `resetToDefaults()` to restore default values.
- Increased default hoop tolerance multiplier (3x → 5x target radius) for more realistic hit detection.
- Added `hoopToleranceMultiplier` to `SolverConfig` for per-solver customization.
- Added fallback angle sweep when analytical vacuum physics solution fails.
- Air resistance and spin effects can now be enabled
- Better convergence for edge-case trajectories near physical limits.
- Also added changelog into the site

## 1.2.7

- Added `minArcHeightMeters` to ShotInput for controlling trajectory apex height.
- Fixed trajectory validation to ensure paths reach the target.
- Removed forced interpolation in ProjectileMotion for smoother trajectory endings.
- Enforced pitch limits from ShotInput in TrajectorySolver.
- Updated ExampleShooter to support `minArcHeightMeters` configuration.
- Made the Changelog more detailed now.

## 1.2.6

- Improved trajectory calculations for 2026 REBUILT compatibility.
- Corrected mathematical formulas for better accuracy.
- Fully functional 2026 example project.

## 1.2.5

- Revamped examples for 2026 REBUILT game.
- Added LED code demonstration.
- Fixed minor bugs and addressed spelling errors.
- Enhanced trajectory calculations for smoother paths.

## 1.2.4

- (WIP) Added SysID support for all mechanical subsystems.
- Updated ports for 2026 game compatibility.
- Introduced new math helper utilities.
- Implemented full trajectory calculation support.

## 1.2.3

- Transitioned library to 2026 game framework.

## 1.2.2

- Enabled pivot simulation functionality.
- Enhanced example project for better usability.

## 1.2.1

- Fixed minor subsystem bugs.
- Updated documentation for clarity.
- Improved README with up-to-date information.
- Added vendor dependency requirements in JSON.

## 1.1.8-1.2.0

- Overhauled simulation and subsystem architecture.
- Integrated Mech3D for advanced visualization.
- Added Motion Magic and Max Motion integration.
- Introduced Photon Vision (simulation not yet supported).

## 1.1.7

- Rework of Encoder Wrapper, to support further simulation

# 1.1.6

- Simulation works!?

# 1.1.5

- Simulation Rework
- Finally Fixed EncoderWrapper 
- Fix alled subsystem simulation
- Fixxed All Subsystems

# 1.1.4 

- Simulation update / patch
- added mech 3d
# 1.1.3

- Minor bug fixes 
- Added javadocs
- Fixxed github-pages site

## 1.1.2

- Fix every gradle error 
- Re-Sync Gh-Pages with Master 

## 1.1.1 

- Fix Github pages not depolying

## 1.1.0

- Added Simulatoin intergratoin to all subsystems 
- Fixxed Subsystem bugs 
- Updated motor Wrapper and encoder wrapper to work with spark maxes 

## 1.0.4 - 1.0.9

- Major Bug fixes and added Simulatoin for Arm and Elevator 

## 1.0.3

- Java doc comments fix and minor bug fixes

## 1.0.1

- Added musicPlayer class

## 1.0.0

- Added Everything