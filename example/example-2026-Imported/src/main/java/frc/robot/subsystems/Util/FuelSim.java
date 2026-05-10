package frc.robot.subsystems.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FuelSim {
    private static final double PERIOD = 0.02; // sec
    private static int subticks = 2;
    private static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
    private static final double FIELD_COR = Math.sqrt(22 / 51.5); // coefficient of restitution with the field
    private static final double FUEL_COR = 0.5; // coefficient of restitution with another fuel
    private static final double ROBOT_COR = 0.1; // coefficient of restitution with a robot
    private static final double FUEL_RADIUS = 0.075;
    private static final double FIELD_LENGTH = 16.51;
    private static final double FIELD_WIDTH = 8.04;
    private static final double FRICTION = 0.1; // proportion of horizontal velocity to lose per second while on ground

    private static final double AIR_DENSITY = 1.204; // kg/m^3 at 20C
    private static final double DRAG_COEFFICIENT = 0.50; // foam ball Cd
    private static final double FUEL_MASS = 0.215; // kg (avg of 0.448-0.5 lbs)
    private static final double CROSS_SECTION_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS; // m^2
    private static final double DRAG_FACTOR = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * CROSS_SECTION_AREA / FUEL_MASS;

    private static FuelSim instance = null;

    private static final Translation3d[] FIELD_XZ_LINE_STARTS = {
            new Translation3d(0, 0, 0),
            new Translation3d(3.96, 1.57, 0),
            new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
            new Translation3d(4.61, 1.57, 0.165),
            new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
            new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
            new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
            new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
            new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165)
    };

    private static final Translation3d[] FIELD_XZ_LINE_ENDS = {
            new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
            new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
            new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
            new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
            new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
            new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
            new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
            new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
            new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0)
    };

    private static class FieldLine {
        double startY, endY;
        double startX, startZ;
        double endX, endZ;
        double vecX, vecZ;
        double vecSqNorm, vecNorm;
        Translation3d normal;
        
        FieldLine(Translation3d start, Translation3d end) {
            startY = start.getY();
            endY = end.getY();
            startX = start.getX();
            startZ = start.getZ();
            endX = end.getX();
            endZ = end.getZ();
            vecX = endX - startX;
            vecZ = endZ - startZ;
            vecSqNorm = vecX * vecX + vecZ * vecZ;
            vecNorm = Math.sqrt(vecSqNorm);
            normal = new Translation3d(-vecZ, 0, vecX).div(vecNorm);
        }
    }

    private static final FieldLine[] FIELD_LINES = new FieldLine[FIELD_XZ_LINE_STARTS.length];
    static {
        for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
            FIELD_LINES[i] = new FieldLine(FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]);
        }
    }

    private class Fuel {
        private double px, py, pz;
        private double vx, vy, vz;

        private Fuel(Translation3d pos, Translation3d vel) {
            this.px = pos.getX();
            this.py = pos.getY();
            this.pz = pos.getZ();
            this.vx = vel.getX();
            this.vy = vel.getY();
            this.vz = vel.getZ();
        }

        private Fuel(Translation3d pos) {
            this(pos, new Translation3d());
        }

        private void update(double dt) {
            if (pz > FUEL_RADIUS) {
                double ax = GRAVITY.getX(), ay = GRAVITY.getY(), az = GRAVITY.getZ();
                double speedSq = vx * vx + vy * vy + vz * vz;
                if (speedSq > 0.0001) {
                    double speed = Math.sqrt(speedSq);
                    double dragAccelMag = DRAG_FACTOR * speedSq;
                    double dragFactor = -dragAccelMag / speed;
                    ax += vx * dragFactor;
                    ay += vy * dragFactor;
                    az += vz * dragFactor;
                }
                
                px += vx * dt + 0.5 * ax * dt * dt;
                py += vy * dt + 0.5 * ay * dt * dt;
                pz += vz * dt + 0.5 * az * dt * dt;
                
                double hvx = vx + ax * dt;
                double hvy = vy + ay * dt;
                double hvz = vz + az * dt;
                
                double nax = GRAVITY.getX(), nay = GRAVITY.getY(), naz = GRAVITY.getZ();
                double newSpeedSq = hvx * hvx + hvy * hvy + hvz * hvz;
                if (newSpeedSq > 0.0001) {
                    double newSpeed = Math.sqrt(newSpeedSq);
                    double newDragAccelMag = DRAG_FACTOR * newSpeedSq;
                    double newDragFactor = -newDragAccelMag / newSpeed;
                    nax += hvx * newDragFactor;
                    nay += hvy * newDragFactor;
                    naz += hvz * newDragFactor;
                }
                
                vx += 0.5 * (ax + nax) * dt;
                vy += 0.5 * (ay + nay) * dt;
                vz += 0.5 * (az + naz) * dt;
            } else {
                px += vx * dt;
                py += vy * dt;
                pz += vz * dt;
            }
            
            if (Math.abs(vz) < 0.05 && pz <= FUEL_RADIUS + 0.03) {
                vz = 0;
                double frictionMult = 1 - FRICTION * dt;
                vx *= frictionMult;
                vy *= frictionMult;
                vz *= frictionMult;
            }
            
            handleFieldCollisions(dt);
        }

        private void handleXZLineCollision(FieldLine line) {
            if (py < line.startY || py > line.endY)
                return; // not within y range
            
            double dx = px - line.startX;
            double dz = pz - line.startZ;
            
            double dot = dx * line.vecX + dz * line.vecZ;
            if (dot < 0 || dot > line.vecSqNorm) return;
            
            double projX = line.startX + line.vecX * (dot / line.vecSqNorm);
            double projZ = line.startZ + line.vecZ * (dot / line.vecSqNorm);
            
            double distSq = (px - projX) * (px - projX) + (pz - projZ) * (pz - projZ);
            if (distSq > FUEL_RADIUS * FUEL_RADIUS) return;
            
            double dist = Math.sqrt(distSq);
            
            // Apply collision response
            double push = FUEL_RADIUS - dist;
            double nx = line.normal.getX();
            double nz = line.normal.getZ();
            
            px += nx * push;
            pz += nz * push;
            
            double velDotNormal = vx * nx + vz * nz;
            if (velDotNormal > 0)
                return; // already moving away from line
                
            double impulse = (1 + FIELD_COR) * velDotNormal;
            vx -= nx * impulse;
            vz -= nz * impulse;
        }

        private void handleFieldCollisions(double dt) {
            // floor and bumps
            for (int i = 0; i < FIELD_LINES.length; i++) {
                handleXZLineCollision(FIELD_LINES[i]);
            }

            // edges
            if (px < FUEL_RADIUS && vx < 0) {
                px = FUEL_RADIUS;
                vx = -FIELD_COR * vx;
            } else if (px > FIELD_LENGTH - FUEL_RADIUS && vx > 0) {
                px = FIELD_LENGTH - FUEL_RADIUS;
                vx = -FIELD_COR * vx;
            }

            if (py < FUEL_RADIUS && vy < 0) {
                py = FUEL_RADIUS;
                vy = -FIELD_COR * vy;
            } else if (py > FIELD_WIDTH - FUEL_RADIUS && vy > 0) {
                py = FIELD_WIDTH - FUEL_RADIUS;
                vy = -FIELD_COR * vy;
            }

            // hubs
            handleHubCollisions(Hub.BLUE_HUB, dt);
            handleHubCollisions(Hub.RED_HUB, dt);
        }

        private void handleHubCollisions(Hub hub, double dt) {
            hub.handleHubInteraction(this, dt);
            hub.fuelCollideSide(this);
            hub.fuelHitNet(this);
        }
        
        public Translation3d getPos() {
            return new Translation3d(px, py, pz);
        }
    }

    private static void handleFuelCollision(Fuel a, Fuel b, double dx, double dy, double dz, double distance) {
        double nx, ny, nz;
        if (distance == 0) {
            nx = 1; ny = 0; nz = 0;
            distance = 1;
        } else {
            nx = dx / distance;
            ny = dy / distance;
            nz = dz / distance;
        }
        
        double dvx = b.vx - a.vx;
        double dvy = b.vy - a.vy;
        double dvz = b.vz - a.vz;
        
        double dot = dvx * nx + dvy * ny + dvz * nz;
        double impulse = 0.5 * (1 + FUEL_COR) * dot;
        
        double intersection = FUEL_RADIUS * 2 - distance;
        double push = intersection / 2;
        
        a.px += nx * push;
        a.py += ny * push;
        a.pz += nz * push;
        
        b.px -= nx * push;
        b.py -= ny * push;
        b.pz -= nz * push;
        
        a.vx += nx * impulse;
        a.vy += ny * impulse;
        a.vz += nz * impulse;
        
        b.vx -= nx * impulse;
        b.vy -= ny * impulse;
        b.vz -= nz * impulse;
    }

    private static void handleFuelCollisions(ArrayList<Fuel> fuels) {
        double thresholdSq = (FUEL_RADIUS * 2) * (FUEL_RADIUS * 2);
        for (int i = 0; i < fuels.size() - 1; i++) {
            Fuel a = fuels.get(i);
            double ax = a.px, ay = a.py, az = a.pz;
            for (int j = i + 1; j < fuels.size(); j++) {
                Fuel b = fuels.get(j);
                double bx = b.px, by = b.py, bz = b.pz;
                
                double dx = ax - bx;
                if (dx > FUEL_RADIUS * 2 || dx < -FUEL_RADIUS * 2) continue;
                double dy = ay - by;
                if (dy > FUEL_RADIUS * 2 || dy < -FUEL_RADIUS * 2) continue;
                double dz = az - bz;
                if (dz > FUEL_RADIUS * 2 || dz < -FUEL_RADIUS * 2) continue;
                
                double distSq = dx * dx + dy * dy + dz * dz;
                if (distSq < thresholdSq) {
                    handleFuelCollision(a, b, dx, dy, dz, Math.sqrt(distSq));
                }
            }
        }
    }

    private ArrayList<Fuel> fuels = new ArrayList<Fuel>();
    private boolean running = false;
    private double lastSimTime = -1;
    private double accumulator = 0.0;
    private boolean loggingEnabled = true;

    // Trajectory comparison tracking
    private Fuel trackedFuel = null;
    private ArrayList<Translation3d> trackedActualPath = new ArrayList<>();
    private Translation3d[] trackedPredictedPath = null;
    private int trackedTickCount = 0;
    private boolean trackedLanded = false; // true once the tracked ball hits the ground
    private boolean trackedPublished = false; // true once we log the trajectory results
    private static final int TRACK_SAMPLE_INTERVAL = 5; 
    private Supplier<Pose2d> robotSupplier = null;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier = null;
    private double robotWidth; // size along the robot's y axis
    private double robotLength; // size along the robot's x axis
    private double bumperHeight;
    private ArrayList<SimIntake> intakes = new ArrayList<>();

    /**
     * Returns a singleton instance of FuelSim
     */
    public static FuelSim getInstance() {
        if (instance == null) {
            instance = new FuelSim();
        }

        return instance;
    }

    /**
     * Clears the field of fuel
     */
    public void clearFuel() {
        fuels.clear();
    }

    /**
     * Spawns fuel in the neutral zone and depots
     */
    public void spawnStartingFuel() {
        // Center fuel
        Translation3d center = new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS);
        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 6; j++) {
                fuels.add(new Fuel(center.plus(new Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i,
                        0))));
                fuels.add(new Fuel(center.plus(new Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i,
                        0))));
                fuels.add(new Fuel(center.plus(new Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i,
                        0))));
                fuels.add(new Fuel(center.plus(new Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i,
                        0))));
            }
        }

        // Depots
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                fuels.add(new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(
                        new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(
                        new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, FUEL_RADIUS)));
            }
        }
    }

    /**
     * Adds array of `Translation3d`'s to NetworkTables at
     * "AdvantageKit/RealOutputs/Fuel Simulation/Fuels"
     */
    public void logFuels() {
        if (!loggingEnabled) return;
        
        Translation3d[] fuelPositions = new Translation3d[fuels.size()];
        for (int i = 0; i < fuels.size(); i++) {
            fuelPositions[i] = fuels.get(i).getPos();
        }
        
        Logger.recordOutput("Fuel Simulation/Fuels", fuelPositions);
        logTrajectoryComparison();
    }

    private void logTrajectoryComparison() {
        if (!loggingEnabled) {
            trackedActualPath.clear();
            trackedPredictedPath = null;
            trackedFuel = null;
            return;
        }
        if (trackedFuel == null) return;

        // Only publish once after landing to reduce logging overhead
        if (!trackedLanded || trackedPublished) return;

        // Log actual sim path so far
        Translation3d[] actualArr = trackedActualPath.toArray(new Translation3d[0]);
        Logger.recordOutput("Fuel Simulation/Comparison/ActualPath", actualArr);

        // Log actual path as arrays for easy plotting
        double[] ax = new double[actualArr.length];
        double[] ay = new double[actualArr.length];
        double[] az = new double[actualArr.length];
        for (int i = 0; i < actualArr.length; i++) {
            ax[i] = actualArr[i].getX();
            ay[i] = actualArr[i].getY();
            az[i] = actualArr[i].getZ();
        }
        Logger.recordOutput("Fuel Simulation/Comparison/Actual_X", ax);
        Logger.recordOutput("Fuel Simulation/Comparison/Actual_Y", ay);
        Logger.recordOutput("Fuel Simulation/Comparison/Actual_Z", az);
        Logger.recordOutput("Fuel Simulation/Comparison/ActualPointCount", actualArr.length);

        // Log predicted path
        if (trackedPredictedPath != null) {
            Logger.recordOutput("Fuel Simulation/Comparison/PredictedPath", trackedPredictedPath);

            double[] px = new double[trackedPredictedPath.length];
            double[] py = new double[trackedPredictedPath.length];
            double[] pz = new double[trackedPredictedPath.length];
            for (int i = 0; i < trackedPredictedPath.length; i++) {
                px[i] = trackedPredictedPath[i].getX();
                py[i] = trackedPredictedPath[i].getY();
                pz[i] = trackedPredictedPath[i].getZ();
            }
            Logger.recordOutput("Fuel Simulation/Comparison/Predicted_X", px);
            Logger.recordOutput("Fuel Simulation/Comparison/Predicted_Y", py);
            Logger.recordOutput("Fuel Simulation/Comparison/Predicted_Z", pz);
            Logger.recordOutput("Fuel Simulation/Comparison/PredictedPointCount", trackedPredictedPath.length);
        }

        // Log endpoint error if both paths have data
        if (trackedPredictedPath != null && trackedPredictedPath.length > 0 && actualArr.length > 0) {
            Translation3d lastActual = actualArr[actualArr.length - 1];
            Translation3d lastPredicted = trackedPredictedPath[trackedPredictedPath.length - 1];
            double endpointError = lastActual.getDistance(lastPredicted);
            Logger.recordOutput("Fuel Simulation/Comparison/EndpointErrorMeters", endpointError);

            // Also compute error at matching indices
            int minLen = Math.min(actualArr.length, trackedPredictedPath.length);
            double[] errors = new double[minLen];
            double maxError = 0;
            for (int i = 0; i < minLen; i++) {
                errors[i] = actualArr[i].getDistance(trackedPredictedPath[i]);
                if (errors[i] > maxError) maxError = errors[i];
            }
            Logger.recordOutput("Fuel Simulation/Comparison/PointErrors", errors);
            Logger.recordOutput("Fuel Simulation/Comparison/MaxErrorMeters", maxError);
        }

        Logger.recordOutput("Fuel Simulation/Comparison/TrackingActive", false);
        trackedPublished = true;
        
        // Clear tracking data to free memory
        trackedActualPath.clear();
        trackedActualPath.trimToSize();
        trackedPredictedPath = null;
        trackedFuel = null;
    }

    /**
     * Start the simulation. `updateSim` must still be called every loop
     */
    public void start() {
        running = true;
    }

    /**
     * Pause the simulation.
     */
    public void stop() {
        running = false;
    }

    /**
     * Sets the number of physics iterations per loop (0.02s)
     * 
     * @param subticks
     */
    public void setSubticks(int subticks) {
        FuelSim.subticks = subticks;
    }

    /**
     * Registers a robot with the fuel simulator
     * 
     * @param width               from left to right (y-axis)
     * @param length              from front to back (x-axis)
     * @param bumperHeight
     * @param poseSupplier
     * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
     */
    public void registerRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.robotSupplier = poseSupplier;
        this.robotSpeedsSupplier = fieldSpeedsSupplier;
        this.robotWidth = width;
        this.robotLength = length;
        this.bumperHeight = bumperHeight;
    }

    /**
     * To be called periodically
     * Will do nothing if sim is not running
     */
    public void updateSim() {
        if (!running) {
            lastSimTime = -1;
            accumulator = 0.0;
            return;
        }

        double currentTime = Timer.getFPGATimestamp();
        if (lastSimTime < 0) {
            lastSimTime = currentTime;
            accumulator = 0.0;
            stepSim(PERIOD);
        } else {
            double dt = currentTime - lastSimTime;
            lastSimTime = currentTime;
            
            // Cap dt to a large value to allow jumping ahead after lag, 
            // but prevent absolute infinity if the computer goes to sleep
            if (dt > 2.0) dt = 2.0;
            
            accumulator += dt;
            while (accumulator >= PERIOD) {
                stepSim(PERIOD);
                accumulator -= PERIOD;
            }
        }

        if (loggingEnabled) {
            logFuels();
        }
    }

    /**
     * Run the simulation forward by a specific time step
     */
    public void stepSim(double dt) {
        double subDt = dt / subticks;
        for (int i = 0; i < subticks; i++) {
            for (Fuel fuel : fuels) {
                fuel.update(subDt);
            }

            handleFuelCollisions(fuels);

            if (robotSupplier != null) {
                handleRobotCollisions(fuels);
                handleIntakes(fuels);
            }

            // Record tracked fuel position at sample interval (stop once it lands)
            if (trackedFuel != null && !trackedLanded) {
                trackedTickCount++;
                if (trackedTickCount % TRACK_SAMPLE_INTERVAL == 0) {
                    if (fuels.contains(trackedFuel)) {
                        trackedActualPath.add(trackedFuel.getPos());
                    } else {
                        trackedLanded = true;
                    }
                }
                // Stop tracking once ball hits the ground
                if (trackedFuel.pz <= FUEL_RADIUS + 0.01 && trackedActualPath.size() > 5) {
                    trackedLanded = true;
                }
            }
        }
    }

    /**
     * Adds a fuel onto the field
     * 
     * @param pos Position to spawn at
     * @param vel Initial velocity vector
     */
    public void spawnFuel(Translation3d pos, Translation3d vel) {
        Fuel fuel = new Fuel(pos, vel);
        fuels.add(fuel);
    }

    /**
     * Adds a fuel onto the field and tracks it for trajectory comparison.
     * 
     * @param pos           Position to spawn at
     * @param vel           Initial velocity vector
     * @param predictedPath Predicted flight path from TrajectorySolver (as Pose3d list)
     */
    public void spawnFuelTracked(Translation3d pos, Translation3d vel, List<Pose3d> predictedPath) {
        Fuel fuel = new Fuel(pos, vel);
        fuels.add(fuel);

        // Start tracking this fuel
        trackedFuel = fuel;
        trackedActualPath.clear();
        trackedActualPath.add(pos); // record initial position
        trackedTickCount = 0;
        trackedLanded = false;
        trackedPublished = false;

        // Convert predicted Pose3d path to Translation3d array
        if (predictedPath != null && !predictedPath.isEmpty()) {
            trackedPredictedPath = new Translation3d[predictedPath.size()];
            for (int i = 0; i < predictedPath.size(); i++) {
                trackedPredictedPath[i] = predictedPath.get(i).getTranslation();
            }
        } else {
            trackedPredictedPath = null;
        }

        if (loggingEnabled) {
            Logger.recordOutput("Fuel Simulation/Comparison/TrackingActive", true);
            Logger.recordOutput("Fuel Simulation/Comparison/LaunchPos",
                    new double[] { pos.getX(), pos.getY(), pos.getZ() });
            Logger.recordOutput("Fuel Simulation/Comparison/LaunchVel",
                    new double[] { vel.getX(), vel.getY(), vel.getZ() });
            Logger.recordOutput("Fuel Simulation/Comparison/LaunchSpeed", vel.getNorm());
        }
    }

    private void handleRobotCollision(Fuel fuel, Pose2d robot, Translation2d robotVel) {
        if (fuel.pz > bumperHeight)
            return; // above bumpers
            
        double dx = fuel.px - robot.getX();
        double dy = fuel.py - robot.getY();
        double cos = robot.getRotation().getCos();
        double sin = robot.getRotation().getSin();
        double relX = dx * cos + dy * sin;
        double relY = -dx * sin + dy * cos;

        double distanceToBottom = -FUEL_RADIUS - robotLength / 2 - relX;
        double distanceToTop = -FUEL_RADIUS - robotLength / 2 + relX;
        double distanceToRight = -FUEL_RADIUS - robotWidth / 2 - relY;
        double distanceToLeft = -FUEL_RADIUS - robotWidth / 2 + relY;

        // not inside robot
        if (distanceToBottom > 0 || distanceToTop > 0 || distanceToRight > 0 || distanceToLeft > 0)
            return;

        Translation2d posOffset;
        // find minimum distance to side and send corresponding collision response
        if ((distanceToBottom >= distanceToTop
                && distanceToBottom >= distanceToRight
                && distanceToBottom >= distanceToLeft)) {
            posOffset = new Translation2d(distanceToBottom, 0);
        } else if ((distanceToTop >= distanceToBottom
                && distanceToTop >= distanceToRight
                && distanceToTop >= distanceToLeft)) {
            posOffset = new Translation2d(-distanceToTop, 0);
        } else if ((distanceToRight >= distanceToBottom
                && distanceToRight >= distanceToTop
                && distanceToRight >= distanceToLeft)) {
            posOffset = new Translation2d(0, distanceToRight);
        } else {
            posOffset = new Translation2d(0, -distanceToLeft);
        }

        posOffset = posOffset.rotateBy(robot.getRotation());
        fuel.px += posOffset.getX();
        fuel.py += posOffset.getY();
        
        Translation2d normal = posOffset.div(posOffset.getNorm());
        double velDotNormal = fuel.vx * normal.getX() + fuel.vy * normal.getY();
        if (velDotNormal < 0) {
            double impulseMag = -velDotNormal * (1 + ROBOT_COR);
            fuel.vx += normal.getX() * impulseMag;
            fuel.vy += normal.getY() * impulseMag;
        }
        
        double robotVelDotNormal = robotVel.dot(normal);
        if (robotVelDotNormal > 0) {
            fuel.vx += normal.getX() * robotVelDotNormal;
            fuel.vy += normal.getY() * robotVelDotNormal;
        }
    }

    private void handleRobotCollisions(ArrayList<Fuel> fuels) {
        Pose2d robot = robotSupplier.get();
        ChassisSpeeds speeds = robotSpeedsSupplier.get();
        Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        for (Fuel fuel : fuels) {
            handleRobotCollision(fuel, robot, robotVel);
        }
    }

    private void handleIntakes(ArrayList<Fuel> fuels) {
        Pose2d robot = robotSupplier.get();
        double cos = robot.getRotation().getCos();
        double sin = robot.getRotation().getSin();
        double rx = robot.getX();
        double ry = robot.getY();
        
        for (SimIntake intake : intakes) {
            if (!intake.ableToIntake.getAsBoolean()) continue;
            for (int i = 0; i < fuels.size(); i++) {
                Fuel fuel = fuels.get(i);
                if (fuel.pz > bumperHeight) continue;
                
                double dx = fuel.px - rx;
                double dy = fuel.py - ry;
                double relX = dx * cos + dy * sin;
                double relY = -dx * sin + dy * cos;
                
                if (relX >= intake.xMin && relX <= intake.xMax && relY >= intake.yMin && relY <= intake.yMax) {
                    intake.callback.run();
                    fuels.remove(i);
                    i--;
                }
            }
        }
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel
     * from the field based on the `ableToIntake` parameter.
     * 
     * @param xMin           Minimum x position for the bounding box
     * @param xMax           Maximum x position for the bounding box
     * @param yMin           Minimum y position for the bounding box
     * @param yMax           Maximum y position for the bounding box
     * @param ableToIntake   Should a return a boolean whether the intake is active
     * @param intakeCallback Function to call when a fuel is intaked
     */
    public void registerIntake(
            double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake, Runnable intakeCallback) {
        intakes.add(new SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback));
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel
     * from the field based on the `ableToIntake` parameter.
     * 
     * @param xMin         Minimum x position for the bounding box
     * @param xMax         Maximum x position for the bounding box
     * @param yMin         Minimum y position for the bounding box
     * @param yMax         Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake) {
        registerIntake(xMin, xMax, yMin, yMax, ableToIntake, () -> {
        });
    }

    /**
     * Registers an intake with the fuel simulator. This intake will always remove
     * fuel from the field.
     * 
     * @param xMin           Minimum x position for the bounding box
     * @param xMax           Maximum x position for the bounding box
     * @param yMin           Minimum y position for the bounding box
     * @param yMax           Maximum y position for the bounding box
     * @param intakeCallback Function to call when a fuel is intaked
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax, Runnable intakeCallback) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, intakeCallback);
    }

    /**
     * Registers an intake with the fuel simulator. This intake will always remove
     * fuel from the field.
     * 
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, () -> {
        });
    }

    public static class Hub {
        public static final Hub BLUE_HUB = new Hub(new Translation2d(4.61, FIELD_WIDTH / 2),
                new Translation3d(5.3, FIELD_WIDTH / 2, 0.89), 1);
        public static final Hub RED_HUB = new Hub(
                new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
                new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
                -1);

        private static final double ENTRY_HEIGHT = 1.83;
        private static final double ENTRY_RADIUS = 0.56;

        private static final double SIDE = 1.2;

        private static final double NET_HEIGHT_MAX = 3.057;
        private static final double NET_HEIGHT_MIN = 1.5;
        private static final double NET_OFFSET = SIDE / 2 + 0.261;
        private static final double NET_WIDTH = 1.484;

        private final Translation2d center;
        private final Translation3d exit;
        private final int exitVelXMult;

        private int score = 0;

        private Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
            this.center = center;
            this.exit = exit;
            this.exitVelXMult = exitVelXMult;
        }

        private void handleHubInteraction(Fuel fuel, double dt) {
            if (didFuelScore(fuel, dt)) {
                fuel.px = exit.getX();
                fuel.py = exit.getY();
                fuel.pz = exit.getZ();
                Translation3d dispersal = getDispersalVelocity();
                fuel.vx = dispersal.getX();
                fuel.vy = dispersal.getY();
                fuel.vz = dispersal.getZ();
                score++;
            }
        }

        private boolean didFuelScore(Fuel fuel, double dt) {
            double pz = fuel.pz;
            if (pz > ENTRY_HEIGHT) return false;
            if (pz - fuel.vz * dt <= ENTRY_HEIGHT) return false;
            
            double dx = fuel.px - center.getX();
            double dy = fuel.py - center.getY();
            return dx * dx + dy * dy <= ENTRY_RADIUS * ENTRY_RADIUS;
        }

        private Translation3d getDispersalVelocity() {
            return new Translation3d(exitVelXMult * (Math.random() + 0.1) * 1.5, Math.random() * 2 - 1, 0);
        }

        /**
         * Reset this hub's score to 0
         */
        public void resetScore() {
            score = 0;
        }

        /**
         * Get the current count of fuel scored in this hub
         * 
         * @return
         */
        public int getScore() {
            return score;
        }

        private void fuelCollideSide(Fuel fuel) {
            if (fuel.pz > ENTRY_HEIGHT - 0.1) return; // above hub
            
            double px = fuel.px;
            double py = fuel.py;
            double cx = center.getX();
            double cy = center.getY();
            
            double distanceToLeft = cx - SIDE / 2 - FUEL_RADIUS - px;
            double distanceToRight = px - cx - SIDE / 2 - FUEL_RADIUS;
            double distanceToTop = cy - SIDE / 2 - FUEL_RADIUS - py;
            double distanceToBottom = py - cy - SIDE / 2 - FUEL_RADIUS;

            // not inside hub
            if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0)
                return;

            // find minimum distance to side and send corresponding collision response
            double colX = 0, colY = 0;
            if (px < cx - SIDE / 2
                    || (distanceToLeft >= distanceToRight
                            && distanceToLeft >= distanceToTop
                            && distanceToLeft >= distanceToBottom)) {
                colX = distanceToLeft;
            } else if (px >= cx + SIDE / 2
                    || (distanceToRight >= distanceToLeft
                            && distanceToRight >= distanceToTop
                            && distanceToRight >= distanceToBottom)) {
                colX = -distanceToRight;
            } else if (py > cy + SIDE / 2
                    || (distanceToTop >= distanceToLeft
                            && distanceToTop >= distanceToRight
                            && distanceToTop >= distanceToBottom)) {
                colY = -distanceToTop;
            } else {
                colY = distanceToBottom;
            }
            
            if (colX != 0) {
                fuel.px += colX;
                fuel.vx = -(1 + FIELD_COR) * fuel.vx;
            } else if (colY != 0) {
                fuel.py += colY;
                fuel.vy = -(1 + FIELD_COR) * fuel.vy;
            }
        }

        private void fuelHitNet(Fuel fuel) {
            double pz = fuel.pz;
            if (pz > NET_HEIGHT_MAX || pz < NET_HEIGHT_MIN) return;
            
            double py = fuel.py;
            double cy = center.getY();
            if (py > cy + NET_WIDTH / 2 || py < cy - NET_WIDTH / 2) return;
            
            double px = fuel.px;
            double cx = center.getX();
            double netX = cx + NET_OFFSET * exitVelXMult;
            
            double netCollision = 0;
            if (px > netX) {
                netCollision = Math.max(0, netX - (px - FUEL_RADIUS));
            } else {
                netCollision = Math.min(0, netX - (px + FUEL_RADIUS));
            }
            
            if (netCollision != 0) {
                fuel.px += netCollision;
                fuel.vx = -fuel.vx * 0.2;
                fuel.vy = fuel.vy * 0.2;
            }
        }
    }

    private class SimIntake {
        double xMin, xMax, yMin, yMax;
        BooleanSupplier ableToIntake;
        Runnable callback;

        private SimIntake(
                double xMin,
                double xMax,
                double yMin,
                double yMax,
                BooleanSupplier ableToIntake,
                Runnable intakeCallback) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
            this.ableToIntake = ableToIntake;
            this.callback = intakeCallback;
        }
    }

    private FuelSim() {
    }

    /**
     * Returns whether logging is enabled
     */
    public boolean isLoggingEnabled() {
        return loggingEnabled;
    }

    /**
     * Enables or disables logging
     * 
     * @param enabled
     */
    public void setLoggingEnabled(boolean enabled) {
        this.loggingEnabled = enabled;
    }
}