# Changelog for  AbsoluteLib V2


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