/**
 * Trajectory Solver Module for FRC Robot Turret Shooting.
 * 
 * <p>This module provides a comprehensive API for calculating optimal shooting
 * trajectories for FRC game pieces. It includes:
 * 
 * <ul>
 *   <li><b>Physics Engine</b> - Projectile motion with optional air resistance</li>
 *   <li><b>Flywheel Modeling</b> - Simulation and generation of flywheel configurations</li>
 *   <li><b>CRT Solver</b> - Chinese Remainder Theorem for discrete angle/RPM solutions</li>
 *   <li><b>Game Piece Specs</b> - Pre-configured game pieces for FRC seasons</li>
 * </ul>
 * 
 * <h2>Quick Start</h2>
 * <pre>{@code
 * // Create solver for 2026 REBUILT game
 * TrajectorySolver solver = TrajectorySolver.forGame2026();
 * 
 * // Define shot parameters
 * ShotInput input = ShotInput.builder()
 *     .shooterPositionMeters(1.0, 2.0, 0.5)
 *     .shooterYawDegrees(45)
 *     .targetPositionMeters(5.0, 5.0, 2.5)
 *     .preferHighArc(true)
 *     .build();
 * 
 * // Solve for trajectory
 * TrajectoryResult result = solver.solve(input);
 * 
 * if (result.isSuccess()) {
 *     double pitch = result.getPitchAngleDegrees();
 *     double rpm = result.getRecommendedRpm();
 *     FlywheelConfig flywheel = result.getRecommendedFlywheel();
 * }
 * }</pre>
 * 
 * <h2>Package Structure</h2>
 * <ul>
 *   <li>{@code physics/} - Physics constants, air resistance, projectile motion</li>
 *   <li>{@code motor/} - FRC motor specifications</li>
 *   <li>{@code gamepiece/} - Game piece definitions</li>
 *   <li>{@code flywheel/} - Flywheel configuration, simulation, and generation</li>
 *   <li>{@code solver/} - Chinese Remainder Theorem solver</li>
 * </ul>
 * 
 * <h2>2026 REBUILT Game Ball Specifications</h2>
 * <ul>
 *   <li>Shape: Sphere (Ball)</li>
 *   <li>Diameter: 5.91 inches</li>
 *   <li>Material: High-density foam</li>
 *   <li>Weight: 0.448-0.5 lbs</li>
 * </ul>
 * 
 * <h2>Supported Motors</h2>
 * <ul>
 *   <li>WCP Kraken X60 (recommended)</li>
 *   <li>REV NEO / NEO Vortex / NEO 550</li>
 *   <li>VEX Falcon 500</li>
 *   <li>CIM / Mini CIM</li>
 *   <li>AndyMark 775pro</li>
 * </ul>
 * 
 * @since 2026
 * @see ca.team4308.absolutelib.math.trajectories.TrajectorySolver
 * @see ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces
 */
package ca.team4308.absolutelib.math.trajectories;
