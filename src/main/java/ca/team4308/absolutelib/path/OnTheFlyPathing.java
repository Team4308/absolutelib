package ca.team4308.absolutelib.path;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;

public final class OnTheFlyPathing {
	private OnTheFlyPathing() {}

	/**
	 * Factory interface that adapts a concrete path library.
	 * @param <P> Path type produced by the library
	 * @param <W> Waypoint type used by the library
	 * @param <C> Constraints/config type used during path construction
	 * @param <G> Goal/end-state type used to encode final heading or holonomic state
	 */
	public interface PathFactory<P, W, C, G> {
		/** Create a list of waypoints from poses (pose heading is direction of travel). */
		List<W> waypointsFromPoses(Pose2d... poses);
		/** Construct a path from waypoints, constraints, start (nullable), and goal end state. */
		P createPath(List<W> waypoints, C constraints, Object idealStart, G goalEndState);
		/** Mark the created path to prevent alliance flipping if coordinates are already correct. */
		void setPreventFlipping(P path, boolean prevent);
	}

	/**
	 * Create a path from poses using the provided factory and constraints.
	 * @param factory Adapter to an actual path library (e.g., PathPlanner)
	 * @param constraints Library-specific constraints object
	 * @param goalEndStateForHeading Library-specific goal/end-state object that captures desired final heading
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

	/** Convenience overload that takes a List of poses. */
	public static <P, W, C, G> P createPathFromPoses(
			PathFactory<P, W, C, G> factory,
			C constraints,
			G goalEndStateForHeading,
			boolean preventFlipping,
			List<Pose2d> poses) {
	Pose2d[] arr = poses.toArray(Pose2d[]::new);
	return createPathFromPoses(factory, constraints, goalEndStateForHeading, preventFlipping, arr);
	}

	/** Create waypoints directly from poses. */
	public static <P, W, C, G> List<W> createWaypointsFromPoses(
			PathFactory<P, W, C, G> factory,
			Pose2d... poses) {
		Objects.requireNonNull(factory, "factory");
		if (poses == null || poses.length < 2) {
			throw new IllegalArgumentException("At least 2 poses are required to form waypoints");
		}
		return factory.waypointsFromPoses(poses);
	}

	/** Build a path from existing waypoints (no explicit start state). */
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
	 * @param factory Adapter to an actual path library (e.g., PathPlanner)
	 * @param constraints Library-specific constraints object
	 * @param idealStart Library-specific start/initial state (nullable)
	 * @param goalEndStateForHeading Library-specific goal/end-state object that captures desired final heading
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


	/** Create a PathPlannerPath directly from poses (pose rotation is the travel heading). */
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

	/** Create a PathPlannerPath from path points (deprecated internal usage). */
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

	/** Direct path between poses using their pose rotations as travel headings. */
	public static PathPlannerPath direct(Pose2d start, Pose2d end,
	                                     PathConstraints constraints,
	                                     Rotation2d endHeading,
	                                     double endVelocityMps,
	                                     boolean preventFlipping) {
		return ppFromPoses(constraints, endHeading, endVelocityMps, preventFlipping, start, end);
	}

	/**
	 * Single-arc path via an offset midpoint. Positive offset = "left", negative = "right"
	 * relative to the start->end direction.
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
	 * S-curve with two midpoints: first offset to the left, second to the right (or vice versa),
	 * producing a smooth "S".
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
	 * Generate a small library of variants you can pick from at runtime.
	 * Keys: "direct", "arcLeft", "arcRight", "sCurve", "slowApproach", "fastApproach"
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

	// ---------------------------------
	// Path selection with a WPILib chooser
	// ---------------------------------

	/** Build a chooser for the variants (defaults to the first key found). */
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

	/** Resolve the currently selected path from a chooser + variants map. */
	public static PathPlannerPath selected(SendableChooser<String> chooser, Map<String, PathPlannerPath> variants) {
		String k = chooser.getSelected();
		return k == null ? null : variants.get(k);
	}

	// ----------------
	// Small utilities
	// ----------------

	/** Convenience factory for constraints (defaults angular + voltage). */
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
