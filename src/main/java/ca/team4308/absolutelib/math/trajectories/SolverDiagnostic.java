package ca.team4308.absolutelib.math.trajectories;

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
}
