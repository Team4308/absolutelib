package ca.team4308.absolutelib.math.trajectories.network;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import ca.team4308.absolutelib.math.trajectories.impl.Pose3d;

public class FullTelemetryPacket {

    @JsonProperty("magic")
    public byte magic = MAGIC_BYTE;
    @JsonProperty("timestamp")
    public double timestamp;
    @JsonProperty("sequence")
    public long sequence;

    // Solution
    @JsonProperty("status")
    public String status = "";
    @JsonProperty("pitch_deg")
    public double pitchDeg;
    @JsonProperty("yaw_deg")
    public double yawDeg;
    @JsonProperty("rpm")
    public double rpm;
    @JsonProperty("exit_vel_mps")
    public double exitVelocityMps;
    @JsonProperty("confidence")
    public double confidence;

    // Flight Path (Max 40 points to avoid IP fragmentation)
    @JsonProperty("path_count")
    public int pathCount;
    @JsonProperty("flight_path_x")
    public double[] flightPathX = new double[0];
    @JsonProperty("flight_path_y")
    public double[] flightPathY = new double[0];
    @JsonProperty("flight_path_z")
    public double[] flightPathZ = new double[0];

    // Flywheel Sim
    @JsonProperty("fw_wheel_rpm")
    public double fwWheelRpm;
    @JsonProperty("fw_motor_rpm")
    public double fwMotorRpm;
    @JsonProperty("fw_motor_power")
    public double fwMotorPower;
    @JsonProperty("fw_spin_up_sec")
    public double fwSpinUpSec;
    @JsonProperty("fw_current_amps")
    public double fwCurrentAmps;
    @JsonProperty("fw_stored_j")
    public double fwStoredJoules;
    @JsonProperty("fw_contact_ms")
    public double fwContactMs;
    @JsonProperty("fw_ball_spin_rpm")
    public double fwBallSpinRpm;
    @JsonProperty("fw_slip_ratio")
    public double fwSlipRatio;
    @JsonProperty("fw_efficiency")
    public double fwEfficiency;
    @JsonProperty("fw_achievable")
    public boolean fwAchievable;
    @JsonProperty("fw_limiting_factor")
    public String fwLimitingFactor = "";

    // Trajectory Metrics
    @JsonProperty("met_tof")
    public double metTof;
    @JsonProperty("met_max_height")
    public double metMaxHeight;
    @JsonProperty("met_margin_error")
    public double metMarginError;
    @JsonProperty("met_dist")
    public double metDistance;
    @JsonProperty("met_height_diff")
    public double metHeightDiff;

    // Solver Trace
    @JsonProperty("tr_mode")
    public String trMode = "";
    @JsonProperty("tr_time_ms")
    public double trTimeMs;
    @JsonProperty("tr_iter")
    public int trIterations;
    @JsonProperty("tr_total_tested")
    public int trTotalTested;
    @JsonProperty("tr_accepted")
    public int trAccepted;
    @JsonProperty("tr_rej_col")
    public int trRejCollision;
    @JsonProperty("tr_rej_low")
    public int trRejArcTooLow;
    @JsonProperty("tr_rej_clear")
    public int trRejClearance;
    @JsonProperty("tr_rej_miss")
    public int trRejMiss;
    @JsonProperty("tr_rej_fly")
    public int trRejFlyover;

    // Discrete Solution
    @JsonProperty("ds_valid")
    public boolean dsValid;
    @JsonProperty("ds_rpm")
    public double dsRpm;
    @JsonProperty("ds_pitch_deg")
    public double dsPitchDeg;
    @JsonProperty("ds_rpm_ticks")
    public int dsRpmTicks;
    @JsonProperty("ds_angle_ticks")
    public int dsAngleTicks;
    @JsonProperty("ds_score")
    public double dsScore;

    // Input Echo
    @JsonProperty("in_rx")
    public double inRobotX;
    @JsonProperty("in_ry")
    public double inRobotY;
    @JsonProperty("in_rz")
    public double inRobotZ;
    @JsonProperty("in_tx")
    public double inTargetX;
    @JsonProperty("in_ty")
    public double inTargetY;
    @JsonProperty("in_tz")
    public double inTargetZ;
    @JsonProperty("in_vx")
    public double inVx;
    @JsonProperty("in_vy")
    public double inVy;

    public static final byte MAGIC_BYTE = 0x55; // U for UDP FullTelemetry

    public FullTelemetryPacket() {}

    public void setFlightPath(List<edu.wpi.first.math.geometry.Pose3d> flightPath) {
        if (flightPath == null || flightPath.isEmpty()) {
            this.pathCount = 0;
            this.flightPathX = new double[0];
            this.flightPathY = new double[0];
            this.flightPathZ = new double[0];
            return;
        }

        int maxPoints = 40;
        int sampleRate = Math.max(1, flightPath.size() / maxPoints);
        
        List<edu.wpi.first.math.geometry.Pose3d> sampled = new ArrayList<>();
        for (int i = 0; i < flightPath.size(); i += sampleRate) {
            if (sampled.size() >= maxPoints) {
                break;
            }
            sampled.add(flightPath.get(i));
        }
        
        if (!flightPath.isEmpty() && sampled.size() < maxPoints) {
            sampled.add(flightPath.get(flightPath.size() - 1));
        }

        this.pathCount = sampled.size();
        this.flightPathX = new double[this.pathCount];
        this.flightPathY = new double[this.pathCount];
        this.flightPathZ = new double[this.pathCount];

        for (int i = 0; i < this.pathCount; i++) {
            edu.wpi.first.math.geometry.Pose3d pose = sampled.get(i);
            this.flightPathX[i] = pose.getX();
            this.flightPathY[i] = pose.getY();
            this.flightPathZ[i] = pose.getZ();
        }
    }
}
