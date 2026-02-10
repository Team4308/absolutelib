/**
 * Hybrid shooter system combining a lookup table, physics solver, RPM feedback,
 * and movement compensation.
 * 
 * <h2>Architecture</h2>
 * <pre>
 *  distance ──► ShotLookupTable ──────┐
 *                                     ├──► ShooterSystem ──► ShotParameters
 *  distance ──► TrajectorySolver ─────┘         │
 *                                               ▼
 *                  RPMCorrector ◄── measuredRpm  │
 *                                               ▼
 *               MovementCompensator ◄── vx, vy  │
 *                                               ▼
 *                 SafetyValidator ──► fire / hold
 * </pre>
 * 
 * <h2>Shot Modes</h2>
 * <ul>
 *   <li>{@link ca.team4308.absolutelib.math.trajectories.shooter.ShotMode#LOOKUP_ONLY LOOKUP_ONLY}
 *       — Fastest and most reliable. Uses pre-tested values only.</li>
 *   <li>{@link ca.team4308.absolutelib.math.trajectories.shooter.ShotMode#SOLVER_ONLY SOLVER_ONLY}
 *       — Physics-based. Most flexible but depends on model accuracy.</li>
 *   <li>{@link ca.team4308.absolutelib.math.trajectories.shooter.ShotMode#LOOKUP_WITH_SOLVER_FALLBACK LOOKUP_WITH_SOLVER_FALLBACK}
 *       — Match-day default. Table for known ranges, solver outside them.</li>
 *   <li>{@link ca.team4308.absolutelib.math.trajectories.shooter.ShotMode#SOLVER_WITH_LOOKUP_FALLBACK SOLVER_WITH_LOOKUP_FALLBACK}
 *       — Good for tuning/testing.</li>
 *   <li>{@link ca.team4308.absolutelib.math.trajectories.shooter.ShotMode#BLENDED BLENDED}
 *       — Weighted average of table and solver.</li>
 *   <li>{@link ca.team4308.absolutelib.math.trajectories.shooter.ShotMode#MANUAL MANUAL}
 *       — Driver override.</li>
 * </ul>
 * 
 * <h2>Quick Start</h2>
 * <pre>{@code
 * // 1. Configure
 * ShooterConfig config = ShooterConfig.defaults2026();
 * ShotLookupTable table = new ShotLookupTable()
 *     .addEntry(1.5, 75.0, 2000)
 *     .addEntry(3.0, 60.0, 3200)
 *     .addEntry(5.0, 50.0, 4200);
 * 
 * // 2. Create (no solver needed for lookup-only)
 * ShooterSystem system = new ShooterSystem(config, table);
 * 
 * // 3. Calculate
 * ShotParameters shot = system.calculate(distance, measuredRpm, vx, vy, yaw);
 * 
 * // 4. Validate & fire
 * if (system.isReadyToFire(measuredRpm)) {
 *     pivot.setAngle(shot.pitchDegrees);
 *     flywheel.spinTo(shot.rpm);
 *     // fire!
 * }
 * }</pre>
 * 
 * @see ca.team4308.absolutelib.math.trajectories.shooter.ShooterSystem
 * @see ca.team4308.absolutelib.math.trajectories.shooter.ShotLookupTable
 * @see ca.team4308.absolutelib.math.trajectories.shooter.ShooterConfig
 */
package ca.team4308.absolutelib.math.trajectories.shooter;
