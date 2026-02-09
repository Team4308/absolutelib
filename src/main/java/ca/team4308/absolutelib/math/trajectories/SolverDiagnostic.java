package ca.team4308.absolutelib.math.trajectories;

import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelGenerator;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelSimulator;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;
import ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion;

/**
 * Standalone diagnostic tool for testing the trajectory solver without
 * running a full robot simulation. Run this class directly to see solver
 * output for various scenarios.
 * 
 * <pre>
 * cd absolutelib
 * ./gradlew build -x test
 * java -cp build/libs/*.jar ca.team4308.absolutelib.math.trajectories.SolverDiagnostic
 * </pre>
 */
public class SolverDiagnostic {
    
    private static final double HUB_X = 4.03;
    private static final double HUB_Y = 4.0;
    private static final double TARGET_Z = 2.1;
    private static final double TARGET_RADIUS = 0.53;
    
    private static final ObstacleConfig HUB_OBSTACLE = ObstacleConfig.builder()
            .position(HUB_X, HUB_Y)
            .baseSize(1.19, 1.19)
            .wallHeight(1.83)
            .upperStructureHeight(0.41)
            .openingDiameter(1.06)
            .enabled(true)
            .build();
    
    public static void main(String[] args) {
        System.out.println("=== Trajectory Solver Diagnostic ===\n");
        
        SolverConstants.setHoopToleranceMultiplier(10000.0);
        SolverConstants.setBasketDescentToleranceMultiplier(6.0);
        SolverConstants.setMinTargetDistanceMeters(0.05);
        SolverConstants.setVelocityBufferMultiplier(1.2);

        System.out.printf("  velocityBuffer=%.2f, maxDragComp=%.2f%n",
            SolverConstants.getVelocityBufferMultiplier(),
            SolverConstants.getDragCompensationMultiplier());
        System.out.printf("  dragComp is distance-scaled: 1.0 at <=%.1fm, %.2f at >=8m%n",
            SolverConstants.getCloseRangeThresholdMeters(),
            SolverConstants.getDragCompensationMultiplier());
        System.out.printf("  closeRangeBuffer=%.2f, closeRangeThreshold=%.1fm%n",
            SolverConstants.getCloseRangeVelocityMultiplier(),
            SolverConstants.getCloseRangeThresholdMeters());
        System.out.printf("  collisionGraceDistance=%.2fm%n%n",
            SolverConstants.getCollisionGraceDistanceMeters());

        GamePiece gamePiece = GamePieces.REBUILT_2026_BALL;
        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.highAccuracy()
                .toBuilder()
                .hoopToleranceMultiplier(10.0)
                .build();
        TrajectorySolver solver = new TrajectorySolver(gamePiece, solverConfig);
        solver.setDebugEnabled(true);
        
        System.out.println("--- Hub Obstacle ---");
        System.out.printf("  Center: (%.2f, %.2f)%n", HUB_X, HUB_Y);
        System.out.printf("  Base: 1.19 x 1.19m -> X range [%.3f, %.3f], Y range [%.3f, %.3f]%n",
            HUB_X - 0.595, HUB_X + 0.595, HUB_Y - 0.595, HUB_Y + 0.595);
        System.out.printf("  Wall height: 1.83m, Total height: %.2fm%n", HUB_OBSTACLE.getTotalHeight());
        System.out.printf("  Opening diameter: 1.06m%n%n");
        
        // Test scenarios
        testScenario(solver, "Close range (0.4m from hub)", 3.612, 4.023, 0.6, true);
        testScenario(solver, "Close range NO collision check", 3.612, 4.023, 0.6, false);
        testScenario(solver, "Medium range (2m)", 2.0, 4.0, 0.6, true);
        testScenario(solver, "Medium range (4m)", 0.5, 4.0, 0.6, true);
        testScenario(solver, "Long range (6m)", -2.0, 4.0, 0.6, true);
        testScenario(solver, "Far range (8m)", -4.0, 4.0, 0.6, true);
        
        System.out.println("\n=== Pitch Sweep Analysis at 4m distance ===");
        analyzePitchSweep(solver, gamePiece, 0.5, 4.0, 0.6);
        
        System.out.println("\n=== Collision Analysis at Close Range ===");
        analyzeCollisionAtCloseRange(3.612, 4.023, 0.6);
        
        System.out.println("\n=== User's Actual Position (2.023, 2.897) ===");
        testScenario(solver, "User position with collision", 2.023, 2.897, 0.6, true);
        testScenario(solver, "User position NO collision", 2.023, 2.897, 0.6, false);
        analyzeCollisionDetail(2.023, 2.897, 0.6);
        
        // Test the exact failing positions from the user's dashboard (pitch 7.5-37.5)
        System.out.println("\n=== Dashboard Failing Positions (7.5-37.5° pitch, like ExampleShooter) ===");
        TrajectorySolver.SolverConfig hwConfig = TrajectorySolver.SolverConfig.defaults()
                .toBuilder().hoopToleranceMultiplier(10.0).build();
        TrajectorySolver hwSolver = new TrajectorySolver(gamePiece, hwConfig);
        hwSolver.setDebugEnabled(true);
        testWithHwLimits(hwSolver, "Pos1: (0.904, 7.398) d=4.6m", 0.904, 7.398, 0.6);
        testWithHwLimits(hwSolver, "Pos2: (1.817, 3.808) d=2.2m", 1.817, 3.808, 0.6);
        testWithHwLimits(hwSolver, "Pos3: (3.241, 4.009) d=0.76m", 3.241, 4.009, 0.6);
        // Same positions without minArcHeight
        System.out.println("\n=== Same positions, NO minArcHeight ===");
        testWithHwLimitsNoArc(hwSolver, "Pos1 noArc", 0.904, 7.398, 0.6);
        testWithHwLimitsNoArc(hwSolver, "Pos2 noArc", 1.817, 3.808, 0.6);
        testWithHwLimitsNoArc(hwSolver, "Pos3 noArc", 3.241, 4.009, 0.6);
        
        // Detailed per-pitch analysis for Pos2 to understand why it fails
        System.out.println("\n=== Detailed Per-Pitch Analysis: Pos2 (1.817, 3.808) d=2.2m ===");
        analyzePerPitch(gamePiece, 1.817, 3.808, 0.6, 7.5, 37.5);
        
        System.out.println("\n=== Detailed Per-Pitch Analysis: Pos1 (0.904, 7.398) d=4.6m ===");
        analyzePerPitch(gamePiece, 0.904, 7.398, 0.6, 7.5, 37.5);
        
        // Verify Pos2 at wider pitch range to confirm CAD solution exists
        System.out.println("\n=== Pos2 with wider pitch (7.5-60°) ===");
        TrajectorySolver wideSolver = new TrajectorySolver(gamePiece, hwConfig);
        wideSolver.setDebugEnabled(true);
        {
            double dx = 4.0 - 1.817, dy = 4.0 - 3.808;
            double dist = Math.sqrt(dx * dx + dy * dy);
            double yaw = Math.atan2(dy, dx);
            ShotInput wideInput = ShotInput.builder()
                    .shooterPositionMeters(1.817, 3.808, 0.6)
                    .shooterYawRadians(yaw)
                    .targetPositionMeters(4.0, 4.0, 2.1)
                    .targetRadiusMeters(0.53)
                    .pitchRangeDegrees(7.5, 60.0)
                    .angleStepDegrees(0.1)
                    .maxCandidates(1000000000)
                    .minArcHeightMeters(0.0)
                    .shotPreference(ShotInput.ShotPreference.AUTO)
                    .addObstacle(HUB_OBSTACLE)
                    .collisionCheckEnabled(true)
                    .build();
            TrajectoryResult r = wideSolver.solve(wideInput);
            if (r.isSuccess()) {
                System.out.printf("  SUCCESS: pitch=%.1f° rpm=%.0f velocity=%.2f maxH=%.2f%n",
                    r.getPitchAngleDegrees(), r.getRecommendedRpm(),
                    r.getRequiredVelocityMps(), r.getMaxHeightMeters());
            } else {
                System.out.printf("  FAILED: %s%n", r.getStatusMessage());
            }
            if (r.getDebugInfo() != null) System.out.println(r.getDebugInfo().getSummary());
        }

        System.out.println("\n=== Velocity Analysis ===");
        analyzeVelocity(solver, gamePiece);
    }
    
    private static void testScenario(TrajectorySolver solver, String name, 
            double shooterX, double shooterY, double shooterZ, boolean collisionEnabled) {
        System.out.println("--- " + name + " ---");
        
        double dx = HUB_X - shooterX;
        double dy = HUB_Y - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        System.out.printf("  Shooter: (%.3f, %.3f, %.3f)  Distance: %.3fm%n", 
            shooterX, shooterY, shooterZ, distance);
        
        boolean insideFootprint = HUB_OBSTACLE.isAboveFootprint(shooterX, shooterY);
        System.out.printf("  Shooter inside hub footprint: %s%n", insideFootprint);
        
        boolean pathCrosses = HUB_OBSTACLE.pathCrossesObstacle(
            shooterX, shooterY, HUB_X, HUB_Y);
        System.out.printf("  Path crosses hub: %s%n", pathCrosses);
        
        ShotInput.Builder inputBuilder = ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterZ)
                .shooterYawRadians(Math.atan2(dy, dx))
                .targetPositionMeters(HUB_X, HUB_Y, TARGET_Z)
                .targetRadiusMeters(TARGET_RADIUS)
                .pitchRangeDegrees(15, 75)
                .minArcHeightMeters(0.75)
                .shotPreference(ShotInput.ShotPreference.HIGH_CLEARANCE)
                .addObstacle(HUB_OBSTACLE)
                .collisionCheckEnabled(collisionEnabled)
                .preferredArcHeightMeters(2.5)
                .arcBiasStrength(0.6);
        
        ShotInput input = inputBuilder.build();
        TrajectoryResult result = solver.solve(input);
        
        if (result.isSuccess()) {
            System.out.printf("  SUCCESS: pitch=%.1f° rpm=%.0f velocity=%.2f m/s%n",
                result.getPitchAngleDegrees(), result.getRecommendedRpm(), result.getRequiredVelocityMps());
            System.out.printf("  TOF=%.3fs maxHeight=%.2fm margin=%.3fm confidence=%.1f%n",
                result.getTimeOfFlightSeconds(), result.getMaxHeightMeters(),
                result.getMarginOfErrorMeters(), result.getConfidenceScore());
            if (result.getFlywheelSimulation() != null) {
                System.out.printf("  Flywheel: exitV=%.2f ballSpin=%.1f achievable=%s%n",
                    result.getFlywheelSimulation().exitVelocityMps,
                    result.getFlywheelSimulation().ballSpinRpm,
                    result.getFlywheelSimulation().isAchievable);
            }
        } else {
            System.out.printf("  FAILED: %s - %s%n", result.getStatus(), result.getStatusMessage());
        }
        if (result.getDebugInfo() != null) {
            System.out.println(result.getDebugInfo().getSummary());
        }
        System.out.println();
    }
    
    private static void testWithHwLimits(TrajectorySolver solver, String name,
            double shooterX, double shooterY, double shooterZ) {
        System.out.println("--- " + name + " ---");
        double dx = 4.0 - shooterX;
        double dy = 4.0 - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double yaw = Math.atan2(dy, dx);
        System.out.printf("  Shooter: (%.3f, %.3f, %.3f) Distance: %.3fm%n",
            shooterX, shooterY, shooterZ, distance);
        
        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterZ)
                .shooterYawRadians(yaw)
                .targetPositionMeters(4.0, 4.0, 2.1)
                .targetRadiusMeters(0.53)
                .pitchRangeDegrees(7.5, 37.5)
                .angleStepDegrees(0.1)
                .maxCandidates(1000000000)
                .minArcHeightMeters(0.75)
                .shotPreference(ShotInput.ShotPreference.AUTO)
                .addObstacle(HUB_OBSTACLE)
                .collisionCheckEnabled(true)
                .preferredArcHeightMeters(2.5)
                .arcBiasStrength(0.6)
                .build();
        
        TrajectoryResult result = solver.solve(input);
        if (result.isSuccess()) {
            System.out.printf("  SUCCESS: pitch=%.1f° rpm=%.0f velocity=%.2f m/s maxH=%.2f%n",
                result.getPitchAngleDegrees(), result.getRecommendedRpm(),
                result.getRequiredVelocityMps(), result.getMaxHeightMeters());
        } else {
            System.out.printf("  FAILED: %s - %s%n", result.getStatus(), result.getStatusMessage());
        }
        if (result.getDebugInfo() != null) {
            System.out.println(result.getDebugInfo().getSummary());
        }
        System.out.println();
    }
    
    private static void testWithHwLimitsNoArc(TrajectorySolver solver, String name,
            double shooterX, double shooterY, double shooterZ) {
        System.out.println("--- " + name + " ---");
        double dx = 4.0 - shooterX;
        double dy = 4.0 - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double yaw = Math.atan2(dy, dx);
        
        ShotInput input = ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterZ)
                .shooterYawRadians(yaw)
                .targetPositionMeters(4.0, 4.0, 2.1)
                .targetRadiusMeters(0.53)
                .pitchRangeDegrees(7.5, 37.5)
                .angleStepDegrees(0.1)
                .maxCandidates(1000000000)
                .minArcHeightMeters(0.0)  // No arc height requirement
                .shotPreference(ShotInput.ShotPreference.AUTO)
                .addObstacle(HUB_OBSTACLE)
                .collisionCheckEnabled(true)
                .build();
        
        TrajectoryResult result = solver.solve(input);
        if (result.isSuccess()) {
            System.out.printf("  SUCCESS: pitch=%.1f° rpm=%.0f velocity=%.2f m/s maxH=%.2f%n",
                result.getPitchAngleDegrees(), result.getRecommendedRpm(),
                result.getRequiredVelocityMps(), result.getMaxHeightMeters());
        } else {
            System.out.printf("  FAILED: %s - %s%n", result.getStatus(), result.getStatusMessage());
        }
        if (result.getDebugInfo() != null) {
            System.out.println(result.getDebugInfo().getSummary());
        }
        System.out.println();
    }

    private static void analyzePitchSweep(TrajectorySolver solver, GamePiece gamePiece,
            double shooterX, double shooterY, double shooterZ) {
        double dx = HUB_X - shooterX;
        double dy = HUB_Y - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double heightDiff = TARGET_Z - shooterZ;
        
        ProjectileMotion pm = new ProjectileMotion();
        double minVelocity = pm.calculateMinimumVelocity(distance, heightDiff);
        double dragComp = TrajectorySolver.calculateDragCompensation(distance);
        double bufferVelocity = minVelocity * SolverConstants.getVelocityBufferMultiplier() * dragComp;
        
        System.out.printf("  Distance: %.2fm, HeightDiff: %.2fm%n", distance, heightDiff);
        System.out.printf("  Min velocity (vacuum): %.2f m/s, With drag+buffer: %.2f m/s%n", minVelocity, bufferVelocity);
        
        double yaw = Math.atan2(dy, dx);
        
        System.out.printf("  %-8s %-10s %-10s %-10s %-12s%n", 
            "Pitch°", "HitTarget", "Closest", "TOF", "MaxHeight");
        
        for (double pitch = 15; pitch <= 75; pitch += 5) {
            ProjectileMotion.TrajectoryResult traj = pm.simulate(
                gamePiece, shooterX, shooterY, shooterZ,
                bufferVelocity, Math.toRadians(pitch), yaw,
                0, HUB_X, HUB_Y, TARGET_Z, TARGET_RADIUS
            );
            
            boolean collides = false;
            for (ProjectileMotion.TrajectoryState state : traj.trajectory) {
                if (HUB_OBSTACLE.checkCollision(state.x, state.y, state.z)) {
                    collides = true;
                    break;
                }
            }
            
            System.out.printf("  %-8.0f %-10s %-10.3f %-10.3f %-12.2f %s%n",
                pitch, traj.hitTarget, traj.closestApproach, traj.flightTime,
                traj.maxHeight, collides ? "COLLIDES" : "clear");
        }
    }
    
    private static void analyzeCollisionAtCloseRange(double shooterX, double shooterY, double shooterZ) {
        double dx = HUB_X - shooterX;
        double dy = HUB_Y - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        
        System.out.printf("  Shooter at (%.3f, %.3f), distance=%.3fm%n", shooterX, shooterY, distance);
        System.out.printf("  Shooter inside hub footprint: %s%n", 
            HUB_OBSTACLE.isAboveFootprint(shooterX, shooterY));
        
        GamePiece gp = GamePieces.REBUILT_2026_BALL;
        ProjectileMotion pm = new ProjectileMotion();
        double yaw = Math.atan2(dy, dx);
        
        System.out.printf("%n  Testing at velocity=15 m/s (reasonable for close range):%n");
        System.out.printf("  %-8s %-10s %-10s %-10s %-10s %-20s%n",
            "Pitch°", "Hit", "Closest", "TOF", "MaxHt", "Collision");
        
        for (double pitch = 15; pitch <= 85; pitch += 5) {
            ProjectileMotion.TrajectoryResult traj = pm.simulate(
                gp, shooterX, shooterY, shooterZ,
                15.0, Math.toRadians(pitch), yaw,
                0, HUB_X, HUB_Y, TARGET_Z, TARGET_RADIUS
            );
            
            String collisionInfo = "clear";
            int collisionCount = 0;
            int firstCollisionIdx = -1;
            for (int i = 0; i < traj.trajectory.length; i++) {
                ProjectileMotion.TrajectoryState s = traj.trajectory[i];
                if (HUB_OBSTACLE.checkCollision(s.x, s.y, s.z)) {
                    collisionCount++;
                    if (firstCollisionIdx < 0) firstCollisionIdx = i;
                }
            }
            if (collisionCount > 0) {
                ProjectileMotion.TrajectoryState s = traj.trajectory[firstCollisionIdx];
                collisionInfo = String.format("HIT @pt%d (%.2f,%.2f,%.2f) [%d pts]", 
                    firstCollisionIdx, s.x, s.y, s.z, collisionCount);
            }
            
            System.out.printf("  %-8.0f %-10s %-10.3f %-10.3f %-10.2f %s%n",
                pitch, traj.hitTarget, traj.closestApproach, traj.flightTime,
                traj.maxHeight, collisionInfo);
        }
    }
    
    private static void analyzeCollisionDetail(double shooterX, double shooterY, double shooterZ) {
        System.out.println("\n  --- Detailed Collision Analysis (ExampleShooter-matched) ---");
        double dx = 4.0 - shooterX;
        double dy = 4.0 - shooterY;
        double distToTarget = Math.sqrt(dx * dx + dy * dy);
        double yaw = Math.atan2(dy, dx);
        
        System.out.printf("  Distance to target (4.0, 4.0, 2.1): %.3fm%n", distToTarget);
        
        // Match ExampleShooter EXACTLY: defaults config, target (4.0,4.0,2.1), radius 0.15
        TrajectorySolver.SolverConfig matchConfig = TrajectorySolver.SolverConfig.defaults()
                .toBuilder()
                .hoopToleranceMultiplier(10.0)
                .build();
        
        GamePiece gp = GamePieces.REBUILT_2026_BALL;
        TrajectorySolver matchSolver = new TrajectorySolver(gp, matchConfig);
        matchSolver.setDebugEnabled(true);
        
        // Exact ExampleShooter ShotInput
        ShotInput matchInput = ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterZ)
                .shooterYawRadians(yaw)
                .targetPositionMeters(4.0, 4.0, 2.1)
                .targetRadiusMeters(0.15)
                .pitchRangeDegrees(0, 90)
                .angleStepDegrees(0.1)
                .maxCandidates(1000000000)
                .shotPreference(ShotInput.ShotPreference.AUTO)
                .addObstacle(HUB_OBSTACLE)
                .collisionCheckEnabled(true)
                .build();
        System.out.printf("  ShotInput: pitch=[%.1f, %.1f] step=%.1f target=%.2fm radius=%.2f%n",
            matchInput.getMinPitchDegrees(), matchInput.getMaxPitchDegrees(),
            matchInput.getAngleStepDegrees(), matchInput.getTargetRadius(),
            matchInput.getTargetRadius());
        System.out.printf("  pathCrossesObstacle: %s%n", matchInput.pathRequiresArc());
        System.out.printf("  effectiveMinPitch with force: %.1f%n",
            matchInput.pathRequiresArc() ? Math.max(matchInput.getMinPitchDegrees(), 
            SolverConstants.getForceHighArcMinPitchDegrees()) : matchInput.getMinPitchDegrees());
        
        // Run full solve and get debug info
        TrajectoryResult matchResult = matchSolver.solve(matchInput);
        System.out.printf("  SOLVE result: %s - %s%n", matchResult.getStatus(), matchResult.getStatusMessage());
        if (matchResult.isSuccess()) {
            System.out.printf("  pitch=%.1f° rpm=%.0f velocity=%.2f m/s%n",
                matchResult.getPitchAngleDegrees(), matchResult.getRecommendedRpm(),
                matchResult.getRequiredVelocityMps());
        }
        if (matchResult.getDebugInfo() != null) {
            System.out.println(matchResult.getDebugInfo().getSummary());
        }
        
        // Now do manual per-pitch collision analysis with VERBOSE logging
        System.out.println("\n  --- Per-Pitch Verbose Collision Check (every 5°) ---");
        ProjectileMotion pm = new ProjectileMotion();
        double minV = pm.calculateMinimumVelocity(distToTarget, 2.1 - shooterZ);
        boolean closeRange = distToTarget < SolverConstants.getCloseRangeThresholdMeters();
        double velocityBuffer = closeRange ? SolverConstants.getCloseRangeVelocityMultiplier()
                : SolverConstants.getVelocityBufferMultiplier() 
                * TrajectorySolver.calculateDragCompensation(distToTarget);
        double targetVelocity = minV * velocityBuffer;
        System.out.printf("  Velocity calc: minV=%.2f buffer=%.2f (closeRange=%s) targetV=%.2f%n",
            minV, velocityBuffer, closeRange, targetVelocity);
        
        // Use same flywheel generation as solver
        FlywheelGenerator gen = new FlywheelGenerator(gp, matchConfig.getFlywheelGenParams());
        FlywheelGenerator.GenerationResult genResult = gen.generateAndEvaluate(targetVelocity);
        double actualVelocity = genResult.bestConfig != null ? genResult.bestConfig.simulation.exitVelocityMps : targetVelocity;
        System.out.printf("  Flywheel: achievable=%d actual exitV=%.2f m/s%n", 
            genResult.achievableCount, actualVelocity);
        
        for (double pitch = 35; pitch <= 85; pitch += 5) {
            ProjectileMotion.TrajectoryResult traj = pm.simulate(
                gp, shooterX, shooterY, shooterZ,
                actualVelocity, Math.toRadians(pitch), yaw,
                genResult.bestConfig != null ? genResult.bestConfig.simulation.ballSpinRpm : 0,
                4.0, 4.0, 2.1, 0.15
            );
            
            System.out.printf("  pitch=%2.0f° pts=%d hit=%s closest=%.3f maxH=%.2f tof=%.3f%n",
                pitch, traj.trajectory.length, traj.hitTarget, traj.closestApproach, 
                traj.maxHeight, traj.flightTime);
            
            boolean collides = TrajectorySolver.trajectoryCollidesInternal(
                traj, matchInput, shooterX, shooterY, true);
            if (!collides) {
                System.out.println("    -> CLEAR (no collision)");
            }
        }
    }
    
    private static void analyzeVelocity(TrajectorySolver solver, GamePiece gamePiece) {
        ProjectileMotion pm = new ProjectileMotion();
        
        double[] distances = {0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0};
        double heightDiff = TARGET_Z - 0.6;
        
        System.out.printf("  %-10s %-12s %-12s %-12s %-10s %-12s%n",
            "Distance", "MinVelocity", "Buffered", "CloseRange?", "Buffer", "DragComp");
        
        for (double d : distances) {
            double minV = pm.calculateMinimumVelocity(d, heightDiff);
            boolean closeRange = d < SolverConstants.getCloseRangeThresholdMeters();
            double buffer;
            double drag;
            if (closeRange) {
                buffer = SolverConstants.getCloseRangeVelocityMultiplier();
                drag = 1.0;
            } else {
                buffer = SolverConstants.getVelocityBufferMultiplier();
                drag = TrajectorySolver.calculateDragCompensation(d);
            }
            double buffered = minV * buffer * drag;
            
            System.out.printf("  %-10.1fm %-12.2f %-12.2f %-12s %-10.2f %-12.2f%n",
                d, minV, buffered, closeRange ? "YES" : "no", buffer, drag);
        }
    }
    
    private static void analyzePerPitch(GamePiece gamePiece, double shooterX, double shooterY, 
            double shooterZ, double minPitch, double maxPitch) {
        double dx = 4.0 - shooterX;
        double dy = 4.0 - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double heightDiff = 2.1 - shooterZ;
        double yaw = Math.atan2(dy, dx);
        double dragComp = TrajectorySolver.calculateDragCompensation(distance);
        boolean isCloseRange = distance < SolverConstants.getCloseRangeThresholdMeters();
        
        System.out.printf("  Distance=%.3fm heightDiff=%.2fm dragComp=%.3f closeRange=%s%n",
            distance, heightDiff, dragComp, isCloseRange);
        
        // Calculate representative velocity (same logic as solver)
        ProjectileMotion pm = new ProjectileMotion();
        double minV = pm.calculateMinimumVelocity(distance, heightDiff);
        double velocityBuffer = isCloseRange ? SolverConstants.getCloseRangeVelocityMultiplier()
            : SolverConstants.getVelocityBufferMultiplier() * dragComp;
        double baseVelocity = minV * velocityBuffer;
        double maxPitchV = TrajectorySolver.calculateRequiredVelocityForPitch(distance, heightDiff, Math.toRadians(maxPitch));
        double representativeV = baseVelocity;
        if (!Double.isNaN(maxPitchV) && maxPitchV > 0) {
            representativeV = Math.max(baseVelocity, maxPitchV * dragComp);
        }
        System.out.printf("  minV=%.2f baseV=%.2f maxPitchV=%.2f representativeV=%.2f%n",
            minV, baseVelocity, Double.isNaN(maxPitchV) ? -1 : maxPitchV, representativeV);
        
        // Generate flywheel for representative velocity
        FlywheelGenerator gen = new FlywheelGenerator(gamePiece);
        FlywheelGenerator.GenerationResult genResult = gen.generateAndEvaluate(representativeV);
        System.out.printf("  Flywheel gen for %.2f: achievable=%d%n", representativeV, genResult.achievableCount);
        if (genResult.achievableCount == 0 && representativeV > baseVelocity) {
            genResult = gen.generateAndEvaluate(baseVelocity);
            System.out.printf("  Fallback gen for %.2f: achievable=%d%n", baseVelocity, genResult.achievableCount);
        }
        if (genResult.achievableCount == 0) {
            genResult = gen.evaluatePresets(representativeV);
            System.out.printf("  Presets for %.2f: achievable=%d%n", representativeV, genResult.achievableCount);
        }
        
        if (genResult.bestConfig == null) {
            System.out.println("  NO FLYWHEEL CONFIG FOUND - cannot continue");
            return;
        }
        
        FlywheelConfig fw = genResult.bestConfig.config;
        FlywheelSimulator sim = new FlywheelSimulator(fw, gamePiece);
        
        // Test max achievable velocity
        FlywheelSimulator.SimulationResult maxSim = sim.simulateMaxVelocity();
        System.out.printf("  Flywheel max exit velocity: %.2f m/s (achievable=%s)%n",
            maxSim.exitVelocityMps, maxSim.isAchievable);
        System.out.printf("  Config: %s wheel=%.1f\" gear=%.2f motor=%s%n",
            fw.getArrangement(), fw.getWheelDiameterInches(), fw.getGearRatio(), fw.getMotor().getName());
        
        // Test each pitch in the valid range
        System.out.printf("  %-8s %-10s %-12s %-12s %-10s%n",
            "Pitch°", "VacuumV", "TargetV", "FW_Ach?", "Status");
        for (double pitch = minPitch; pitch <= maxPitch; pitch += 0.5) {
            double vacuumV = TrajectorySolver.calculateRequiredVelocityForPitch(distance, heightDiff, Math.toRadians(pitch));
            if (Double.isNaN(vacuumV) || vacuumV <= 0) {
                if (pitch % 5 < 0.1) { // Print every 5° for NaN
                    System.out.printf("  %-8.1f %-10s %-12s %-12s %-10s%n", pitch, "NaN", "-", "-", "phys_impossible");
                }
                continue;
            }
            double targetV = vacuumV * dragComp;
            FlywheelSimulator.SimulationResult pitchSim = sim.simulateForVelocity(targetV);
            if (!pitchSim.isAchievable) {
                System.out.printf("  %-8.1f %-10.2f %-12.2f %-12s %-10s%n", 
                    pitch, vacuumV, targetV, "NO", "fw_unachievable: " + pitchSim.limitingFactor);
            } else {
                // Run trajectory simulation
                ProjectileMotion.TrajectoryResult traj = pm.simulate(
                    gamePiece, shooterX, shooterY, shooterZ,
                    pitchSim.exitVelocityMps, Math.toRadians(pitch), yaw,
                    pitchSim.ballSpinRpm, 4.0, 4.0, 2.1, 0.53
                );
                System.out.printf("  %-8.1f %-10.2f %-12.2f %-12s hit=%s closest=%.3f maxH=%.2f%n",
                    pitch, vacuumV, targetV, "YES", traj.hitTarget, traj.closestApproach, traj.maxHeight);
            }
        }
    }
}
