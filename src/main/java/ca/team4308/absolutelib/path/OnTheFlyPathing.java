package ca.team4308.absolutelib.path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public final class OnTheFlyPathing {

    private OnTheFlyPathing() {
    }

    /**
     * Factory interface that adapts a concrete path library.
     *
     * @param <P> Path type produced by the library
     * @param <W> Waypoint type used by the library
     * @param <C> Constraints/config type used during path construction
     * @param <G> Goal/end-state type used to encode final heading or holonomic
     * state
     */
    public interface PathFactory<P, W, C, G> {

        /**
         * Create a list of waypoints from poses (pose heading is direction of
         * travel).
         */
        List<W> waypointsFromPoses(Pose2d... poses);

        /**
         * Construct a path from waypoints, constraints, start (nullable), and
         * goal end state.
         */
        P createPath(List<W> waypoints, C constraints, Object idealStart, G goalEndState);

        /**
         * Mark the created path to prevent alliance flipping if coordinates are
         * already correct.
         */
        void setPreventFlipping(P path, boolean prevent);
    }

    /**
     * Create a path from poses using the provided factory and constraints.
     *
     * @param factory Adapter to an actual path library (e.g., PathPlanner)
     * @param constraints Library-specific constraints object
     * @param goalEndStateForHeading Library-specific goal/end-state object that
     * captures desired final heading
     * @param preventFlipping If true, disables alliance flipping on the path
     * @param poses Waypoint poses (heading is direction of travel)
     * @return Library-specific path object
     */
    public static <P, W, C, G> P createPathFromPoses(
            PathFactory<P, W, C, G> factory,
            C constraints,
            G goalEndStateForHeading,
            boolean preventFlipping,
            Pose2d... poses) {
        Objects.requireNonNull(factory, "factory");
        Objects.requireNonNull(constraints, "constraints");
        Objects.requireNonNull(goalEndStateForHeading, "goalEndStateForHeading");
        if (poses == null || poses.length < 2) {
            throw new IllegalArgumentException("At least 2 poses are required to form a path");
        }
        List<W> waypoints = factory.waypointsFromPoses(poses);
        P path = factory.createPath(waypoints, constraints, null, goalEndStateForHeading);
        factory.setPreventFlipping(path, preventFlipping);
        return path;
    }

    /**
     * Convenience overload that takes a List of poses.
     */
    public static <P, W, C, G> P createPathFromPoses(
            PathFactory<P, W, C, G> factory,
            C constraints,
            G goalEndStateForHeading,
            boolean preventFlipping,
            List<Pose2d> poses) {
        Pose2d[] arr = poses.toArray(Pose2d[]::new);
        return createPathFromPoses(factory, constraints, goalEndStateForHeading, preventFlipping, arr);
    }

    /**
     * Create waypoints directly from poses.
     */
    public static <P, W, C, G> List<W> createWaypointsFromPoses(
            PathFactory<P, W, C, G> factory,
            Pose2d... poses) {
        Objects.requireNonNull(factory, "factory");
        if (poses == null || poses.length < 2) {
            throw new IllegalArgumentException("At least 2 poses are required to form waypoints");
        }
        return factory.waypointsFromPoses(poses);
    }

    public static <P, W, C, G> P createPathFromWaypoints(
            PathFactory<P, W, C, G> factory,
            C constraints,
            G goalEndStateForHeading,
            boolean preventFlipping,
            List<W> waypoints) {
        Objects.requireNonNull(factory, "factory");
        Objects.requireNonNull(constraints, "constraints");
        Objects.requireNonNull(goalEndStateForHeading, "goalEndStateForHeading");
        Objects.requireNonNull(waypoints, "waypoints");
        if (waypoints.size() < 2) {
            throw new IllegalArgumentException("At least 2 waypoints are required to form a path");
        }
        P path = factory.createPath(waypoints, constraints, null, goalEndStateForHeading);
        factory.setPreventFlipping(path, preventFlipping);
        return path;
    }

    /**
     * Create a path from poses and an optional library-specific start state.
     *
     * @param factory Adapter to an actual path library (e.g., PathPlanner)
     * @param constraints Library-specific constraints object
     * @param idealStart Library-specific start/initial state (nullable)
     * @param goalEndStateForHeading Library-specific goal/end-state object that
     * captures desired final heading
     * @param preventFlipping If true, disables alliance flipping on the path
     * @param poses Waypoint poses (heading is direction of travel)
     */
    public static <P, W, C, G> P createPathFromPosesWithStart(
            PathFactory<P, W, C, G> factory,
            C constraints,
            Object idealStart,
            G goalEndStateForHeading,
            boolean preventFlipping,
            Pose2d... poses) {
        Objects.requireNonNull(factory, "factory");
        Objects.requireNonNull(constraints, "constraints");
        Objects.requireNonNull(goalEndStateForHeading, "goalEndStateForHeading");
        if (poses == null || poses.length < 2) {
            throw new IllegalArgumentException("At least 2 poses are required to form a path");
        }
        List<W> waypoints = factory.waypointsFromPoses(poses);
        P path = factory.createPath(waypoints, constraints, idealStart, goalEndStateForHeading);
        factory.setPreventFlipping(path, preventFlipping);
        return path;
    }

    /**
     * Create a PathPlannerPath directly from poses (pose rotation is the travel
     * heading).
     */
    public static PathPlannerPath ppFromPoses(PathConstraints constraints,
            Rotation2d endHeading,
            double endVelocityMps,
            boolean preventFlipping,
            Pose2d... poses) {
        if (poses == null || poses.length < 2) {
            throw new IllegalArgumentException("At least 2 poses required");
        }
        // Waypoints derive headings from pose rotations; final heading enforced via GoalEndState
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(Arrays.asList(poses));
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null,
                new GoalEndState(endVelocityMps, endHeading));
        path.preventFlipping = preventFlipping;
        return path;
    }

    /**
     * Create a PathPlannerPath from path points (deprecated internal usage).
     */
    public static PathPlannerPath ppFromPoints(PathConstraints constraints,
            Rotation2d endHeading,
            double endVelocityMps,
            boolean preventFlipping,
            List<PathPoint> points) {
        PathPlannerPath path = PathPlannerPath.fromPathPoints(points,
                constraints,
                new GoalEndState(endVelocityMps, endHeading));
        path.preventFlipping = preventFlipping;
        return path;
    }

    /**
     * Direct path between poses using their pose rotations as travel headings.
     */
    public static PathPlannerPath direct(Pose2d start, Pose2d end,
            PathConstraints constraints,
            Rotation2d endHeading,
            double endVelocityMps,
            boolean preventFlipping) {
        return ppFromPoses(constraints, endHeading, endVelocityMps, preventFlipping, start, end);
    }

    /**
     * Single-arc path via an offset midpoint. Positive offset = "left",
     * negative = "right" relative to the start->end direction.
     */
    public static PathPlannerPath arcViaOffsetMidpoint(Pose2d start, Pose2d end,
            double offsetMeters,
            PathConstraints constraints,
            Rotation2d endHeading,
            double endVelocityMps,
            boolean preventFlipping) {
        Translation2d a = start.getTranslation();
        Translation2d b = end.getTranslation();
        Translation2d mid = a.interpolate(b, 0.5);
        Rotation2d dir = b.minus(a).getAngle();
        Translation2d offset = new Translation2d(offsetMeters, 0)
                .rotateBy(dir.plus(Rotation2d.fromDegrees(90)));
        Translation2d midOffset = mid.plus(offset);
        Pose2d startPose = new Pose2d(a, dir);
        Pose2d midPose = new Pose2d(midOffset, b.minus(midOffset).getAngle());
        Pose2d endPose = new Pose2d(b, endHeading);

        return ppFromPoses(constraints, endHeading, endVelocityMps, preventFlipping,
                startPose, midPose, endPose);
    }

    /**
     * S-curve with two midpoints: first offset to the left, second to the right
     * (or vice versa), producing a smooth "S".
     */
    public static PathPlannerPath sCurve(Pose2d start, Pose2d end,
            double firstOffsetMeters, double secondOffsetMeters,
            PathConstraints constraints,
            Rotation2d endHeading,
            double endVelocityMps,
            boolean preventFlipping) {
        Translation2d a = start.getTranslation();
        Translation2d b = end.getTranslation();
        Rotation2d dir = b.minus(a).getAngle();

        Translation2d mid1 = a.interpolate(b, 1.0 / 3.0)
                .plus(new Translation2d(firstOffsetMeters, 0)
                        .rotateBy(dir.plus(Rotation2d.fromDegrees(90))));
        Translation2d mid2 = a.interpolate(b, 2.0 / 3.0)
                .plus(new Translation2d(secondOffsetMeters, 0)
                        .rotateBy(dir.plus(Rotation2d.fromDegrees(90))));

        Pose2d startPose = new Pose2d(a, dir);
        Pose2d midPose1 = new Pose2d(mid1, mid2.minus(mid1).getAngle());
        Pose2d midPose2 = new Pose2d(mid2, b.minus(mid2).getAngle());
        Pose2d endPose = new Pose2d(b, endHeading);

        return ppFromPoses(constraints, endHeading, endVelocityMps, preventFlipping,
                startPose, midPose1, midPose2, endPose);
    }

    /**
     * Generate a small library of variants you can pick from at runtime. Keys:
     * "direct", "arcLeft", "arcRight", "sCurve", "slowApproach",
     * "fastApproach", "corner", "manhattanX", "manhattanY", "auto"
     */
    public static Map<String, PathPlannerPath> variants(Pose2d start, Pose2d end,
            PathConstraints defaultConstraints,
            Rotation2d endHeading,
            boolean preventFlipping) {
        Map<String, PathPlannerPath> map = new HashMap<>();

        map.put("direct", direct(start, end, defaultConstraints, endHeading, 0.0, preventFlipping));
        map.put("arcLeft", arcViaOffsetMidpoint(start, end, +0.75, defaultConstraints, endHeading, 0.0, preventFlipping));
        map.put("arcRight", arcViaOffsetMidpoint(start, end, -0.75, defaultConstraints, endHeading, 0.0, preventFlipping));
        map.put("sCurve", sCurve(start, end, +0.6, -0.6, defaultConstraints, endHeading, 0.0, preventFlipping));
        map.put("corner", corner(start, end, defaultConstraints, endHeading, 0.0, preventFlipping));
        map.put("manhattanX", manhattan(start, end, true, defaultConstraints, endHeading, 0.0, preventFlipping));
        map.put("manhattanY", manhattan(start, end, false, defaultConstraints, endHeading, 0.0, preventFlipping));
        map.put("auto", auto(start, end, defaultConstraints, endHeading, 0.0, preventFlipping));

        double baseVel = defaultConstraints.maxVelocityMPS();
        double baseAccel = defaultConstraints.maxAccelerationMPSSq();
        double baseAngVel = defaultConstraints.maxAngularVelocityRadPerSec();
        double baseAngAccel = defaultConstraints.maxAngularAccelerationRadPerSecSq();
        double voltage = defaultConstraints.nominalVoltageVolts();

        PathConstraints slow = new PathConstraints(
                Math.min(baseVel, 2.0),
                Math.min(baseAccel, 2.0),
                baseAngVel,
                baseAngAccel,
                voltage,
                false
        );
        PathConstraints fast = new PathConstraints(
                baseVel * 1.25,
                baseAccel * 1.25,
                baseAngVel * 1.25,
                baseAngAccel * 1.25,
                voltage,
                false
        );

        map.put("slowApproach", direct(start, end, slow, endHeading, 0.0, preventFlipping));
        map.put("fastApproach", direct(start, end, fast, endHeading, 0.0, preventFlipping));

        return map;
    }

    /**
     * Automatically chooses between "direct" and "corner" based on geometry. If
     * the start and end headings suggest a sharp turn (L-shape) and the
     * intersection is valid, it uses "corner". Otherwise, it uses "direct".
     */
    public static PathPlannerPath auto(Pose2d start, Pose2d end,
            PathConstraints constraints,
            Rotation2d endHeading,
            double endVelocityMps,
            boolean preventFlipping) {
        Translation2d p1 = start.getTranslation();
        Translation2d v1 = new Translation2d(1.0, start.getRotation());
        Translation2d p2 = end.getTranslation();
        Translation2d v2 = new Translation2d(1.0, end.getRotation());

        double det = v1.getX() * v2.getY() - v1.getY() * v2.getX();

        // If parallel or close to parallel, use direct (spline handles S-curves well)
        if (Math.abs(det) < 0.1) { // ~5.7 degrees
            return direct(start, end, constraints, endHeading, endVelocityMps, preventFlipping);
        }

        Translation2d dP = p2.minus(p1);
        double t = (dP.getX() * v2.getY() - dP.getY() * v2.getX()) / det;
        double u = (dP.getX() * v1.getY() - dP.getY() * v1.getX()) / det;

        // Check if intersection is "between" the points in a way that makes a nice corner
        // t > 0: intersection is in front of start
        // u < 0: intersection is behind end (so we approach end from front)
        if (t > 0.5 && u < -0.5) {
            // Check if intersection is not too far away (e.g. < 2x direct distance)
            Translation2d intersection = p1.plus(v1.times(t));
            double directDist = p1.getDistance(p2);
            if (intersection.getDistance(p1) + intersection.getDistance(p2) < directDist * 2.0) {
                return corner(start, end, constraints, endHeading, endVelocityMps, preventFlipping);
            }
        }

        return direct(start, end, constraints, endHeading, endVelocityMps, preventFlipping);
    }

    /**
     * Creates a path that moves to the intersection of the start heading and
     * end heading. If lines are parallel or intersection is invalid, falls back
     * to direct.
     */
    public static PathPlannerPath corner(Pose2d start, Pose2d end,
            PathConstraints constraints,
            Rotation2d endHeading,
            double endVelocityMps,
            boolean preventFlipping) {
        Translation2d p1 = start.getTranslation();
        Translation2d v1 = new Translation2d(1.0, start.getRotation());
        Translation2d p2 = end.getTranslation();
        Translation2d v2 = new Translation2d(1.0, end.getRotation());

        double det = v1.getX() * v2.getY() - v1.getY() * v2.getX();

        if (Math.abs(det) < 1e-3) {
            return direct(start, end, constraints, endHeading, endVelocityMps, preventFlipping);
        }

        Translation2d dP = p2.minus(p1);
        double t = (dP.getX() * v2.getY() - dP.getY() * v2.getX()) / det;
        double u = (dP.getX() * v1.getY() - dP.getY() * v1.getX()) / det;

        // t is distance from start to intersection along v1
        // u is distance from end to intersection along v2
        if (t <= 0.1 || u >= -0.1) {
            return direct(start, end, constraints, endHeading, endVelocityMps, preventFlipping);
        }

        Translation2d intersection = p1.plus(v1.times(t));

        // Check if intersection is far
        if (intersection.getDistance(p1) > p1.getDistance(p2) * 2.5) {
            return direct(start, end, constraints, endHeading, endVelocityMps, preventFlipping);
        }

        Pose2d cornerPose = new Pose2d(intersection, end.getTranslation().minus(intersection).getAngle());
        return ppFromPoses(constraints, endHeading, endVelocityMps, preventFlipping, start, cornerPose, end);
    }

    /**
     * Manhattan-style path: moves along X then Y (or Y then X).
     *
     * @param xFirst if true, moves along X axis first.
     */
    public static PathPlannerPath manhattan(Pose2d start, Pose2d end,
            boolean xFirst,
            PathConstraints constraints,
            Rotation2d endHeading,
            double endVelocityMps,
            boolean preventFlipping) {
        Translation2d mid;
        if (xFirst) {
            mid = new Translation2d(end.getX(), start.getY());
        } else {
            mid = new Translation2d(start.getX(), end.getY());
        }

        Rotation2d midRot = end.getTranslation().minus(mid).getAngle();
        Pose2d midPose = new Pose2d(mid, midRot);

        return ppFromPoses(constraints, endHeading, endVelocityMps, preventFlipping, start, midPose, end);
    }

    /**
     * Creates an arc path around a center point.
     *
     * @param center The center of rotation.
     * @param clockwise True for clockwise orbit, false for counter-clockwise.
     */
    public static PathPlannerPath orbit(Pose2d start, Pose2d end,
            Translation2d center,
            boolean clockwise,
            PathConstraints constraints,
            Rotation2d endHeading,
            double endVelocityMps,
            boolean preventFlipping) {
        Translation2d startVec = start.getTranslation().minus(center);
        Translation2d endVec = end.getTranslation().minus(center);
        double radius = startVec.getNorm();

        double startAngle = startVec.getAngle().getRadians();
        double endAngle = endVec.getAngle().getRadians();

        // Normalize angles
        while (endAngle - startAngle > Math.PI) {
            endAngle -= 2 * Math.PI;
        }
        while (endAngle - startAngle < -Math.PI) {
            endAngle += 2 * Math.PI;
        }

        // Adjust for direction
        if (clockwise && endAngle > startAngle) {
            endAngle -= 2 * Math.PI;
        }
        if (!clockwise && endAngle < startAngle) {
            endAngle += 2 * Math.PI;
        }

        int segments = 8;
        List<Pose2d> poses = new ArrayList<>();
        poses.add(start);

        for (int i = 1; i < segments; i++) {
            double alpha = (double) i / segments;
            double angle = startAngle + alpha * (endAngle - startAngle);
            Translation2d pos = center.plus(new Translation2d(radius, new Rotation2d(angle)));

            // Heading is tangent to the circle
            Rotation2d heading = new Rotation2d(angle + (clockwise ? -Math.PI / 2 : Math.PI / 2));
            poses.add(new Pose2d(pos, heading));
        }

        poses.add(end);

        return ppFromPoses(constraints, endHeading, endVelocityMps, preventFlipping, poses.toArray(new Pose2d[0]));
    }

    // ---------------------------------
    // Path selection with a WPILib chooser
    // ---------------------------------
    /**
     * Build a chooser for the variants (defaults to the first key found).
     */
    public static SendableChooser<String> buildChooser(Map<String, PathPlannerPath> variants, String defaultKey) {
        SendableChooser<String> chooser = new SendableChooser<>();
        boolean setDefault = false;
        for (Map.Entry<String, PathPlannerPath> e : variants.entrySet()) {
            String key = e.getKey();
            if (!setDefault && (defaultKey == null || defaultKey.equals(key))) {
                chooser.setDefaultOption(key, key);
                setDefault = true;
            } else {
                chooser.addOption(key, key);
            }
        }
        return chooser;
    }

    /**
     * Resolve the currently selected path from a chooser + variants map.
     */
    public static PathPlannerPath selected(SendableChooser<String> chooser, Map<String, PathPlannerPath> variants) {
        String k = chooser.getSelected();
        return k == null ? null : variants.get(k);
    }

    // ----------------
    // Small utilities
    // ----------------
    /**
     * Convenience factory for constraints (defaults angular + voltage).
     */
    public static PathConstraints constraints(double maxVelMps, double maxAccelMps2) {
        return new PathConstraints(
                maxVelMps,
                maxAccelMps2,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                12.0,
                false
        );
    }
}
