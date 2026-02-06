package ca.team4308.absolutelib.math.trajectories.physics;

import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;

/**
 * Projectile motion calculator supporting both ideal and air-resistance models.
 * Uses numerical integration for accurate trajectory simulation.
 */
public class ProjectileMotion {
    
    private final AirResistance airResistance;
    private final double timeStep;
    
    /**
     * State vector for trajectory simulation.
     */
    public static class TrajectoryState {
        public double x, y, z;       
        public double vx, vy, vz;    
        public double time;          
        
        public TrajectoryState(double x, double y, double z, 
                               double vx, double vy, double vz, double time) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.vx = vx;
            this.vy = vy;
            this.vz = vz;
            this.time = time;
        }
        
        public TrajectoryState copy() {
            return new TrajectoryState(x, y, z, vx, vy, vz, time);
        }
        
        public double getSpeed() {
            return Math.sqrt(vx * vx + vy * vy + vz * vz);
        }
        
        public double getHorizontalDistance() {
            return Math.sqrt(x * x + y * y);
        }
        
        @Override
        public String toString() {
            return String.format("t=%.3fs pos=(%.2f, %.2f, %.2f)m vel=(%.2f, %.2f, %.2f)m/s",
                time, x, y, z, vx, vy, vz);
        }
    }
    
    /**
     * Result of trajectory simulation.
     */
    public static class TrajectoryResult {
        public final TrajectoryState[] trajectory;
        public final boolean hitTarget;
        public final double closestApproach;
        public final TrajectoryState finalState;
        public final double maxHeight;
        public final double flightTime;
        
        public TrajectoryResult(TrajectoryState[] trajectory, boolean hitTarget,
                                double closestApproach, TrajectoryState finalState,
                                double maxHeight, double flightTime) {
            this.trajectory = trajectory;
            this.hitTarget = hitTarget;
            this.closestApproach = closestApproach;
            this.finalState = finalState;
            this.maxHeight = maxHeight;
            this.flightTime = flightTime;
        }
    }
    
    /**
     * Creates projectile motion calculator with default air resistance.
     */
    public ProjectileMotion() {
        this(new AirResistance(), PhysicsConstants.DEFAULT_TIME_STEP);
    }
    
    /**
     * Creates projectile motion calculator with specified air resistance model.
     */
    public ProjectileMotion(AirResistance airResistance) {
        this(airResistance, PhysicsConstants.DEFAULT_TIME_STEP);
    }
    
    /**
     * Creates projectile motion calculator with full customization.
     */
    public ProjectileMotion(AirResistance airResistance, double timeStep) {
        this.airResistance = airResistance;
        this.timeStep = timeStep;
    }
    
    /**
     * Simulates a projectile trajectory from initial conditions.
     * 
     * @param gamePiece The game piece being shot
     * @param x0 Initial X position (m)
     * @param y0 Initial Y position (m)
     * @param z0 Initial Z position (m)
     * @param velocity Exit velocity (m/s)
     * @param pitchAngle Pitch angle from horizontal (radians)
     * @param yawAngle Yaw angle from +X axis (radians)
     * @param spinRpm Backspin rate in RPM
     * @param targetX Target X position (m)
     * @param targetY Target Y position (m)
     * @param targetZ Target Z position (m)
     * @param targetRadius Acceptable hit radius (m)
     * @return Trajectory simulation result
     */
    public TrajectoryResult simulate(GamePiece gamePiece,
            double x0, double y0, double z0,
            double velocity, double pitchAngle, double yawAngle,
            double spinRpm,
            double targetX, double targetY, double targetZ, double targetRadius) {
        
        double horizontalVelocity = velocity * Math.cos(pitchAngle);
        double vx = horizontalVelocity * Math.cos(yawAngle);
        double vy = horizontalVelocity * Math.sin(yawAngle);
        double vz = velocity * Math.sin(pitchAngle);
        TrajectoryState state = new TrajectoryState(x0, y0, z0, vx, vy, vz, 0);
        
        // Backspin axis is perpendicular to launch yaw
        double spinAxisX = -Math.sin(yawAngle);
        double spinAxisY = Math.cos(yawAngle);
        double spinAxisZ = 0;
        
        int maxPoints = (int)(PhysicsConstants.MAX_FLIGHT_TIME / 0.01) + 1;
        TrajectoryState[] trajectory = new TrajectoryState[maxPoints];
        int pointCount = 0;
        int sampleInterval = (int)(0.01 / timeStep);
        int stepCount = 0;
        
        double maxHeight = z0;
        double closestApproach = Double.MAX_VALUE;
        boolean hitTarget = false;
        boolean pastApex = false;  // Track if ball is descending
        double prevZ = z0;
        TrajectoryState closestState = null;
        
        while (state.time < PhysicsConstants.MAX_FLIGHT_TIME && state.z >= 0) {
            if (stepCount % sampleInterval == 0 && pointCount < maxPoints) {
                trajectory[pointCount++] = state.copy();
            }
            if (state.z > maxHeight) {
                maxHeight = state.z;
            }
            
            // Detect when ball starts descending (past apex)
            if (state.z < prevZ && !pastApex) {
                pastApex = true;
            }
            prevZ = state.z;
            
            double dx = state.x - targetX;
            double dy = state.y - targetY;
            double dz = state.z - targetZ;
            double distToTarget = Math.sqrt(dx * dx + dy * dy + dz * dz);
            double horizontalDistToTarget = Math.sqrt(dx * dx + dy * dy);
            
            if (distToTarget < closestApproach) {
                closestApproach = distToTarget;
                closestState = state.copy();
            }
            
            if (distToTarget <= targetRadius) {
                hitTarget = true;
            }
            
            // For basket-style goals: Stop simulation when ball descends to/below target height
            // while being horizontally close to the target
            if (pastApex && state.z <= targetZ && horizontalDistToTarget < targetRadius * 5) {
                // Ball has descended into the goal zone - this is a valid landing
                hitTarget = true;
                break;
            }
            
            state = integrateRK4(gamePiece, state, timeStep, spinRpm, spinAxisX, spinAxisY, spinAxisZ);
            stepCount++;
        }
        
        TrajectoryState[] trimmedTrajectory = new TrajectoryState[pointCount];
        System.arraycopy(trajectory, 0, trimmedTrajectory, 0, pointCount);
        
        // Final validation: check if the trajectory actually reached close to the target
        // The closest approach must be within a reasonable distance for a valid hit
        if (!hitTarget && closestApproach <= targetRadius * 3) {
            hitTarget = true; // Close enough counts as a hit
        }
        
        return new TrajectoryResult(trimmedTrajectory, hitTarget, closestApproach, 
            state, maxHeight, state.time);
    }

    /**
     * Simulates a projectile trajectory from initial conditions (no spin).
     */
    public TrajectoryResult simulate(GamePiece gamePiece,
            double x0, double y0, double z0,
            double velocity, double pitchAngle, double yawAngle,
            double targetX, double targetY, double targetZ, double targetRadius) {
        return simulate(gamePiece, x0, y0, z0, velocity, pitchAngle, yawAngle, 0, targetX, targetY, targetZ, targetRadius);
    }
    
    /**
     * Calculates the required launch angle for a given distance and velocity.
     * Uses the high-arc solution (larger of two possible angles).
     * 
     * @param distance Horizontal distance to target (m)
     * @param heightDiff Target height minus launch height (m)
     * @param velocity Launch velocity (m/s)
     * @return Launch angle in radians, or NaN if no solution exists
     */
    public double calculateHighArcAngle(double distance, double heightDiff, double velocity) {
        double g = PhysicsConstants.GRAVITY;
        double v2 = velocity * velocity;
        double v4 = v2 * v2;
        
        double discriminant = v4 - g * (g * distance * distance + 2 * heightDiff * v2);
        
        if (discriminant < 0) {
            return Double.NaN; 
        }
        
        double sqrtDisc = Math.sqrt(discriminant);
        
        // High arc solution
        double tanThetaHigh = (v2 + sqrtDisc) / (g * distance);
        return Math.atan(tanThetaHigh);
    }
    
    /**
     * Calculates the low-arc launch angle.
     */
    public double calculateLowArcAngle(double distance, double heightDiff, double velocity) {
        double g = PhysicsConstants.GRAVITY;
        double v2 = velocity * velocity;
        double v4 = v2 * v2;
        
        double discriminant = v4 - g * (g * distance * distance + 2 * heightDiff * v2);
        
        if (discriminant < 0) {
            return Double.NaN;
        }
        
        double sqrtDisc = Math.sqrt(discriminant);
        
        // Low arc solution
        double tanThetaLow = (v2 - sqrtDisc) / (g * distance);
        return Math.atan(tanThetaLow);
    }
    
    /**
     * Calculates minimum velocity required to reach a target.
     * 
     * @param distance Horizontal distance (m)
     * @param heightDiff Height difference (m)
     * @return Minimum required velocity (m/s)
     */
    public double calculateMinimumVelocity(double distance, double heightDiff) {
        double g = PhysicsConstants.GRAVITY;
        
        // Minimum velocity occurs at 45 degrees for level ground
        // For elevated targets: v_min = sqrt(g * (h + sqrt(h^2 + d^2)))
        // where h = heightDiff, d = distance/
        // - Some dude from StackOverflow (????) idk
        
        return Math.sqrt(g * (heightDiff + Math.sqrt(heightDiff * heightDiff + distance * distance)));
    }
    
    /**
     * Calculates time of flight for ideal projectile motion.
     */
    public double calculateTimeOfFlight(double distance, double launchAngle, double velocity) {
        double horizontalVelocity = velocity * Math.cos(launchAngle);
        if (Math.abs(horizontalVelocity) < 1e-6) {
            return Double.POSITIVE_INFINITY;
        }
        return distance / horizontalVelocity;
    }
    
    /**
     * Calculates maximum height achieved in flight.
     */
    public double calculateMaxHeight(double launchHeight, double launchAngle, double velocity) {
        double vz = velocity * Math.sin(launchAngle);
        double g = PhysicsConstants.GRAVITY;
        
        // Max height = h0 + vz^2 / (2g)
        return launchHeight + (vz * vz) / (2 * g);
    }
    
    /**
     * Performs RK4 integration step for accurate trajectory simulation.
     */
    private TrajectoryState integrateRK4(GamePiece gamePiece, TrajectoryState state, double dt, 
                                         double spinRpm, double sax, double say, double saz) {
        // k1
        double[] a1 = calculateAcceleration(gamePiece, state.vx, state.vy, state.vz, spinRpm, sax, say, saz);
        
        // k2
        double vx2 = state.vx + 0.5 * dt * a1[0];
        double vy2 = state.vy + 0.5 * dt * a1[1];
        double vz2 = state.vz + 0.5 * dt * a1[2];
        double[] a2 = calculateAcceleration(gamePiece, vx2, vy2, vz2, spinRpm, sax, say, saz);
        
        // k3
        double vx3 = state.vx + 0.5 * dt * a2[0];
        double vy3 = state.vy + 0.5 * dt * a2[1];
        double vz3 = state.vz + 0.5 * dt * a2[2];
        double[] a3 = calculateAcceleration(gamePiece, vx3, vy3, vz3, spinRpm, sax, say, saz);
        
        // k4
        double vx4 = state.vx + dt * a3[0];
        double vy4 = state.vy + dt * a3[1];
        double vz4 = state.vz + dt * a3[2];
        double[] a4 = calculateAcceleration(gamePiece, vx4, vy4, vz4, spinRpm, sax, say, saz);
        
        // Weighted average
        double ax = (a1[0] + 2*a2[0] + 2*a3[0] + a4[0]) / 6.0;
        double ay = (a1[1] + 2*a2[1] + 2*a3[1] + a4[1]) / 6.0;
        double az = (a1[2] + 2*a2[2] + 2*a3[2] + a4[2]) / 6.0;
        
        // Update velocity
        double newVx = state.vx + dt * ax;
        double newVy = state.vy + dt * ay;
        double newVz = state.vz + dt * az;
        
        // Update position (average velocity method)
        double avgVx = (state.vx + newVx) / 2.0;
        double avgVy = (state.vy + newVy) / 2.0;
        double avgVz = (state.vz + newVz) / 2.0;
        
        double newX = state.x + dt * avgVx;
        double newY = state.y + dt * avgVy;
        double newZ = state.z + dt * avgVz;
        
        return new TrajectoryState(newX, newY, newZ, newVx, newVy, newVz, state.time + dt);
    }
    
    /**
     * Calculates acceleration from gravity, drag, and Magnus effect.
     */
    private double[] calculateAcceleration(GamePiece gamePiece, double vx, double vy, double vz,
                                           double spinRpm, double sax, double say, double saz) {
        double[] dragAccel = airResistance.calculateDragAcceleration(gamePiece, vx, vy, vz);
        
        double ax = dragAccel[0];
        double ay = dragAccel[1];
        double az = dragAccel[2] - PhysicsConstants.GRAVITY;
        
        if (spinRpm != 0) {
            double[] magnusForce = airResistance.calculateMagnusForce(gamePiece, vx, vy, vz, spinRpm, sax, say, saz);
            double mass = gamePiece.getMassKg();
            ax += magnusForce[0] / mass;
            ay += magnusForce[1] / mass;
            az += magnusForce[2] / mass;
        }
        
        return new double[]{ ax, ay, az };
    }
    
    /**
     * Iteratively solves for the launch angle needed to hit a target,
     * accounting for air resistance and magnus effect.
     * 
     * @param gamePiece The game piece
     * @param x0 Launch X position (m)
     * @param y0 Launch Y position (m)
     * @param z0 Launch Z position (m)
     * @param velocity Launch velocity (m/s)
     * @param targetX Target X (m)
     * @param targetY Target Y (m)
     * @param targetZ Target Z (m)
     * @param preferHighArc Whether to prefer high-arc solutions
     * @param spinRpm Backspin RPM (optional, 0 for no spin)
     * @return Optimal pitch angle in radians, or NaN if no solution
     */
    public double solveForAngle(GamePiece gamePiece,
            double x0, double y0, double z0, double velocity,
            double targetX, double targetY, double targetZ,
            double targetRadius,
            boolean preferHighArc, double spinRpm) {
        
        double horizontalDistance = Math.sqrt(
            Math.pow(targetX - x0, 2) + Math.pow(targetY - y0, 2));
        double heightDiff = targetZ - z0;
        double yawAngle = Math.atan2(targetY - y0, targetX - x0);
        
        double angle = preferHighArc ? 
            calculateHighArcAngle(horizontalDistance, heightDiff, velocity) :
            calculateLowArcAngle(horizontalDistance, heightDiff, velocity);
        
        if (Double.isNaN(angle)) {
            return Double.NaN;
        }
        
        if (!airResistance.isEnabled()) {
            return angle;
        }
        
        double angleLow = 0.1;
        double angleHigh = Math.PI / 2 - 0.1;
        
        if (preferHighArc) {
            angleLow = angle * 0.8;
            angleHigh = Math.PI / 2 - 0.05;
        } else {
            angleLow = 0.05;
            angleHigh = angle * 1.2;
        }
        
        for (int i = 0; i < PhysicsConstants.MAX_ITERATIONS * 2; i++) {
            angle = (angleLow + angleHigh) / 2.0;
            
            TrajectoryResult result = simulate(gamePiece, x0, y0, z0, 
                velocity, angle, yawAngle, spinRpm, targetX, targetY, targetZ, targetRadius);
            
            if (result.hitTarget) {
                return angle;
            }

            double minHorizDist = Double.MAX_VALUE;
            TrajectoryState closestState = null;
            
            for (TrajectoryState state : result.trajectory) {
                if (state == null) break;
                double horizDist = Math.sqrt(
                    Math.pow(state.x - targetX, 2) + Math.pow(state.y - targetY, 2));
                if (horizDist < minHorizDist) {
                    minHorizDist = horizDist;
                    closestState = state;
                }
            }
            
            if (closestState == null) {
                break;
            }
            
            if (closestState.z > targetZ) {
                angleHigh = angle;
            } else {
                angleLow = angle;
            }
            
            if (angleHigh - angleLow < 0.00001) {
                break; 
            }
        }
        
        return angle;
    }

    public double solveForAngle(GamePiece gamePiece,
            double x0, double y0, double z0, double velocity,
            double targetX, double targetY, double targetZ,
            boolean preferHighArc) {
        return solveForAngle(gamePiece, x0, y0, z0, velocity, targetX, targetY, targetZ, 0.05, preferHighArc, 0);
    }
    
    /**
     * Result of evaluating a single angle candidate.
     */
    public static class AngleEvaluation {
        public final double pitchRadians;
        public final double velocity;
        public final TrajectoryResult trajectory;
        public final boolean hitsTarget;
        public final double closestApproach;
        public final double timeOfFlight;
        public final double maxHeight;
        
        public AngleEvaluation(double pitchRadians, double velocity, TrajectoryResult trajectory) {
            this.pitchRadians = pitchRadians;
            this.velocity = velocity;
            this.trajectory = trajectory;
            this.hitsTarget = trajectory.hitTarget;
            this.closestApproach = trajectory.closestApproach;
            this.timeOfFlight = trajectory.flightTime;
            this.maxHeight = trajectory.maxHeight;
        }
        
        public double getPitchDegrees() {
            return Math.toDegrees(pitchRadians);
        }
    }
    
    /**
     * Finds all valid launch angles that can potentially hit the target.
     * Iterates through angle range in O(n) time where n = (maxAngle - minAngle) / stepSize.
     * 
     * @param gamePiece The game piece being shot
     * @param x0 Launch X position (m)
     * @param y0 Launch Y position (m)
     * @param z0 Launch Z position (m)
     * @param velocity Launch velocity (m/s)
     * @param targetX Target X (m)
     * @param targetY Target Y (m)
     * @param targetZ Target Z (m)
     * @param targetRadius Hit acceptance radius (m)
     * @param spinRpm Backspin RPM (0 for no spin)
     * @param minAngleDegrees Minimum pitch angle to try
     * @param maxAngleDegrees Maximum pitch angle to try
     * @param angleStepDegrees Step size for angle iteration
     * @return Array of angle evaluations for all tested angles
     */
    public AngleEvaluation[] findAllAngles(GamePiece gamePiece,
            double x0, double y0, double z0, double velocity,
            double targetX, double targetY, double targetZ,
            double targetRadius, double spinRpm,
            double minAngleDegrees, double maxAngleDegrees, double angleStepDegrees) {
        
        double yawAngle = Math.atan2(targetY - y0, targetX - x0);
        
        int numSteps = (int)Math.ceil((maxAngleDegrees - minAngleDegrees) / angleStepDegrees) + 1;
        AngleEvaluation[] results = new AngleEvaluation[numSteps];
        int count = 0;
        
        for (double angleDeg = minAngleDegrees; angleDeg <= maxAngleDegrees && count < numSteps; angleDeg += angleStepDegrees) {
            double angleRad = Math.toRadians(angleDeg);
            
            TrajectoryResult trajResult = simulate(gamePiece, x0, y0, z0,
                velocity, angleRad, yawAngle, spinRpm,
                targetX, targetY, targetZ, targetRadius);
            
            results[count++] = new AngleEvaluation(angleRad, velocity, trajResult);
        }
        
        // Trim array to actual size
        if (count < results.length) {
            AngleEvaluation[] trimmed = new AngleEvaluation[count];
            System.arraycopy(results, 0, trimmed, 0, count);
            return trimmed;
        }
        
        return results;
    }
    
    /**
     * Calculates the required velocity to hit a target at a given angle.
     * Uses analytical solution for ideal projectile motion.
     * 
     * @param distance Horizontal distance to target (m)
     * @param heightDiff Height difference (target - launch) (m)
     * @param pitchRadians Launch angle in radians
     * @return Required velocity (m/s), or NaN if impossible
     */
    public double calculateRequiredVelocity(double distance, double heightDiff, double pitchRadians) {
        double g = PhysicsConstants.GRAVITY;
        double sinTheta = Math.sin(pitchRadians);
        double cosTheta = Math.cos(pitchRadians);
        double tanTheta = sinTheta / cosTheta;
        
        // From projectile equations:
        
        // heightDiff = distance * tan(theta) - g * distance^2 / (2 * v^2 * cos^2(theta))
        // Solving for v:
        // v^2 = g * distance^2 / (2 * cos^2(theta) * (distance * tan(theta) - heightDiff))
        
        double denominator = 2 * cosTheta * cosTheta * (distance * tanTheta - heightDiff);
        
        if (denominator <= 0) {
            return Double.NaN; // Angle too low or target unreachable at this angle
        }
        
        double v2 = g * distance * distance / denominator;
        return Math.sqrt(v2);
    }
    
    /**
     * Finds all valid angles with their required velocities.
     * For each angle, calculates the minimum velocity needed to reach target.
     * This is O(n) where n = (maxAngle - minAngle) / stepSize.
     * 
     * @param gamePiece The game piece being shot
     * @param x0 Launch X position (m)
     * @param y0 Launch Y position (m)
     * @param z0 Launch Z position (m)
     * @param targetX Target X (m)
     * @param targetY Target Y (m)
     * @param targetZ Target Z (m)
     * @param targetRadius Hit acceptance radius (m)
     * @param spinRpm Backspin RPM (0 for no spin)
     * @param minAngleDegrees Minimum pitch angle to try
     * @param maxAngleDegrees Maximum pitch angle to try
     * @param angleStepDegrees Step size for angle iteration
     * @param minVelocity Minimum allowed velocity (m/s)
     * @param maxVelocity Maximum allowed velocity (m/s)
     * @return Array of valid angle evaluations
     */
    public AngleEvaluation[] findAllAnglesWithVelocity(GamePiece gamePiece,
            double x0, double y0, double z0,
            double targetX, double targetY, double targetZ,
            double targetRadius, double spinRpm,
            double minAngleDegrees, double maxAngleDegrees, double angleStepDegrees,
            double minVelocity, double maxVelocity) {
        
        double horizontalDistance = Math.sqrt(
            Math.pow(targetX - x0, 2) + Math.pow(targetY - y0, 2));
        double heightDiff = targetZ - z0;
        double yawAngle = Math.atan2(targetY - y0, targetX - x0);
        
        int numSteps = (int)Math.ceil((maxAngleDegrees - minAngleDegrees) / angleStepDegrees) + 1;
        AngleEvaluation[] tempResults = new AngleEvaluation[numSteps];
        int validCount = 0;
        
        for (double angleDeg = minAngleDegrees; angleDeg <= maxAngleDegrees; angleDeg += angleStepDegrees) {
            double angleRad = Math.toRadians(angleDeg);
            
            // Calculate ideal required velocity for this angle
            double idealVelocity = calculateRequiredVelocity(horizontalDistance, heightDiff, angleRad);
            
            if (Double.isNaN(idealVelocity) || idealVelocity < minVelocity || idealVelocity > maxVelocity) {
                continue; // Skip invalid angles
            }
            
            double velocity = idealVelocity;
            if (airResistance.isEnabled()) {
                // Change !?
                velocity = idealVelocity * 1.15;
                if (velocity > maxVelocity) {
                    velocity = maxVelocity;
                }
            }
            
            // Simulate to verify
            TrajectoryResult trajResult = simulate(gamePiece, x0, y0, z0,
                velocity, angleRad, yawAngle, spinRpm,
                targetX, targetY, targetZ, targetRadius);
            
            tempResults[validCount++] = new AngleEvaluation(angleRad, velocity, trajResult);
        }
        
        AngleEvaluation[] results = new AngleEvaluation[validCount];
        System.arraycopy(tempResults, 0, results, 0, validCount);
        return results;
    }
    
    public AirResistance getAirResistance() {
        return airResistance;
    }
    
    public double getTimeStep() {
        return timeStep;
    }
}
