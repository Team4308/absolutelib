package ca.team4308.absolutelib.math.trajectories.network;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.ArrayList;
import java.util.List;

/**
 * Data Transfer Object for full trajectory configuration synchronization.
 * Includes hardware limits, solver settings, and lookup table data.
 */
public class TrajectoryConfigDTO {

    @JsonProperty("config_version_id")
    public int configVersionId = 1;

    @JsonProperty("timestamp")
    public double timestamp = 0;

    // --- ShooterConfig Fields ---
    @JsonProperty("shooter_pitch_min")
    public double shooterPitchMin;

    @JsonProperty("shooter_pitch_max")
    public double shooterPitchMax;

    @JsonProperty("shooter_rpm_min")
    public double shooterRpmMin;

    @JsonProperty("shooter_rpm_max")
    public double shooterRpmMax;

    @JsonProperty("shooter_rpm_to_velocity_factor")
    public double shooterRpmToVelocityFactor;

    @JsonProperty("shooter_distance_min")
    public double shooterDistanceMin;

    @JsonProperty("shooter_distance_max")
    public double shooterDistanceMax;

    @JsonProperty("shooter_rpm_feedback_threshold")
    public double shooterRpmFeedbackThreshold;

    @JsonProperty("shooter_rpm_abort_threshold")
    public double shooterRpmAbortThreshold;

    @JsonProperty("shooter_pitch_correction_per_rpm")
    public double shooterPitchCorrectionPerRpm;

    @JsonProperty("shooter_moving_comp_gain")
    public double shooterMovingCompGain;

    @JsonProperty("shooter_moving_iterations")
    public int shooterMovingIterations;

    @JsonProperty("shooter_safety_max_exit_vel")
    public double shooterSafetyMaxExitVel;

    @JsonProperty("shooter_rpm_drop_recovery")
    public double shooterRpmDropRecovery;

    @JsonProperty("shooter_mode")
    public String shooterMode;

    @JsonProperty("shooter_blend_factor")
    public double shooterBlendFactor;

    @JsonProperty("shooter_height")
    public double shooterHeightMeters;

    // --- SolverConfig Fields ---
    @JsonProperty("solver_rpm_tolerance")
    public double solverRpmTolerance;

    @JsonProperty("solver_angle_tolerance")
    public double solverAngleTolerance;

    @JsonProperty("solver_hoop_multiplier")
    public double solverHoopMultiplier;

    @JsonProperty("solver_sweep_step")
    public double solverSweepStep;

    @JsonProperty("solver_vel_refine_iters")
    public int solverVelRefineIters;

    @JsonProperty("solver_sim_step")
    public double solverSimStep;

    @JsonProperty("solver_fast_sim_step")
    public double solverFastSimStep;

    @JsonProperty("solver_use_parallel")
    public boolean solverUseParallel;

    @JsonProperty("solver_mode")
    public String solverSolveMode;

    // --- Lookup Table Fields ---
    @JsonProperty("lookup_distances")
    public double[] lookupDistances = new double[0];

    @JsonProperty("lookup_pitches")
    public double[] lookupPitches = new double[0];

    @JsonProperty("lookup_rpms")
    public double[] lookupRpms = new double[0];

    @JsonProperty("lookup_tofs")
    public double[] lookupTofs = new double[0];

    public TrajectoryConfigDTO() {}

    public static TrajectoryConfigDTO fromFull(
            ca.team4308.absolutelib.math.trajectories.shooter.ShooterConfig sc,
            ca.team4308.absolutelib.math.trajectories.TrajectorySolver.SolverConfig slvc,
            ca.team4308.absolutelib.math.trajectories.shooter.ShotLookupTable table,
            ca.team4308.absolutelib.math.trajectories.shooter.ShotMode mode,
            ca.team4308.absolutelib.math.trajectories.TrajectorySolver.SolveMode solveMode,
            double blendFactor,
            double shooterHeight) {
        
        TrajectoryConfigDTO dto = new TrajectoryConfigDTO();
        dto.shooterHeightMeters = shooterHeight;
        
        // ShooterConfig
        dto.shooterPitchMin = sc.getMinPitchDegrees();
        dto.shooterPitchMax = sc.getMaxPitchDegrees();
        dto.shooterRpmMin = sc.getMinRpm();
        dto.shooterRpmMax = sc.getMaxRpm();
        dto.shooterRpmToVelocityFactor = sc.getRpmToVelocityFactor();
        dto.shooterDistanceMin = sc.getMinDistanceMeters();
        dto.shooterDistanceMax = sc.getMaxDistanceMeters();
        dto.shooterRpmFeedbackThreshold = sc.getRpmFeedbackThreshold();
        dto.shooterRpmAbortThreshold = sc.getRpmAbortThreshold();
        dto.shooterPitchCorrectionPerRpm = sc.getPitchCorrectionPerRpmDeficit();
        dto.shooterMovingCompGain = sc.getMovingCompensationGain();
        dto.shooterMovingIterations = sc.getMovingIterations();
        dto.shooterSafetyMaxExitVel = sc.getSafetyMaxExitVelocity();
        dto.shooterRpmDropRecovery = sc.getRpmDropRecoveryBoost();
        dto.shooterMode = mode.name();
        dto.shooterBlendFactor = blendFactor;

        // SolverConfig
        dto.solverRpmTolerance = slvc.getRpmTolerance();
        dto.solverAngleTolerance = slvc.getAngleTolerance();
        dto.solverHoopMultiplier = slvc.getHoopToleranceMultiplier();
        dto.solverSweepStep = slvc.getSweepStepDegrees();
        dto.solverVelRefineIters = slvc.getVelocityRefineIterations();
        dto.solverSimStep = slvc.getSimulationTimeStep();
        dto.solverFastSimStep = slvc.getFastSimulationTimeStep();
        dto.solverUseParallel = slvc.useParallel();
        dto.solverSolveMode = solveMode.name();

        // Lookup Table
        if (table != null && table.hasEntries()) {
            int size = table.getSize();
            dto.lookupDistances = new double[size];
            dto.lookupPitches = new double[size];
            dto.lookupRpms = new double[size];
            dto.lookupTofs = new double[size];

            int i = 0;
            java.util.TreeSet<Double> distances = new java.util.TreeSet<>(table.getPitchMap().keySet());
            for (Double d : distances) {
                dto.lookupDistances[i] = d;
                dto.lookupPitches[i] = table.getPitchMap().get(d);
                dto.lookupRpms[i] = table.getRpmMap().get(d);
                dto.lookupTofs[i] = table.getTofMap().get(d) != null ? table.getTofMap().get(d) : 0;
                i++;
            }
        }

        return dto;
    }
}
