package ca.team4308.coprocessor;

import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;

public class OutputSmoother {

    private final double alpha;
    private final double resetThresholdDeg;

    private boolean initialized = false;
    private double emaPitch = 0.0;
    private double emaYaw = 0.0;
    private double emaRpm = 0.0;

    public OutputSmoother(double alpha, double resetThresholdDeg) {
        this.alpha = alpha;
        this.resetThresholdDeg = resetThresholdDeg;
    }

    public void process(TrajectoryResponse response) {
        if (!response.valid) {
            initialized = false;
            return;
        }

        if (!initialized) {
            emaPitch = response.pitchDegrees;
            emaYaw = response.yawDegrees;
            emaRpm = response.rpm;
            initialized = true;
            return;
        }

        // Check for large jumps that demand a reset
        double pitchDiff = Math.abs(response.pitchDegrees - emaPitch);
        double yawDiff = Math.abs(response.yawDegrees - emaYaw);

        if (pitchDiff > resetThresholdDeg || yawDiff > resetThresholdDeg) {
            // Target moved significantly or solver jumped to a disconnected curve
            emaPitch = response.pitchDegrees;
            emaYaw = response.yawDegrees;
            emaRpm = response.rpm;
        } else {
            // Apply EMA
            emaPitch = (alpha * response.pitchDegrees) + ((1.0 - alpha) * emaPitch);
            emaYaw = (alpha * response.yawDegrees) + ((1.0 - alpha) * emaYaw);
            emaRpm = (alpha * response.rpm) + ((1.0 - alpha) * emaRpm);
        }

        response.pitchDegrees = emaPitch;
        response.yawDegrees = emaYaw;
        response.rpm = emaRpm;
    }

    public void reset() {
        initialized = false;
    }
}
