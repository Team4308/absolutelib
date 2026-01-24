package ca.team4308.absolutelib.math.trajectories.motor;

/**
 * Specifications for an FRC motor.
 * Provides electrical and mechanical characteristics needed for flywheel calculations.
 */
public class MotorSpec {
    
    private final String name;
    private final double freeSpeedRpm;
    private final double stallTorqueNm;
    private final double freeCurrentAmps;
    private final double stallCurrentAmps;
    private final double nominalVoltage;
    private final double rotorInertiaKgM2;
    private final double mass;
    
    /**
     * Creates a motor specification.
     * 
     * @param name Motor name/model
     * @param freeSpeedRpm Free speed in RPM
     * @param stallTorqueNm Stall torque in Newton-meters
     * @param freeCurrentAmps Free current in Amps
     * @param stallCurrentAmps Stall current in Amps
     * @param nominalVoltage Nominal voltage (typically 12V)
     * @param rotorInertiaKgM2 Rotor moment of inertia in kg*m^2
     * @param mass Motor mass in kg
     */
    public MotorSpec(String name, double freeSpeedRpm, double stallTorqueNm,
                     double freeCurrentAmps, double stallCurrentAmps,
                     double nominalVoltage, double rotorInertiaKgM2, double mass) {
        this.name = name;
        this.freeSpeedRpm = freeSpeedRpm;
        this.stallTorqueNm = stallTorqueNm;
        this.freeCurrentAmps = freeCurrentAmps;
        this.stallCurrentAmps = stallCurrentAmps;
        this.nominalVoltage = nominalVoltage;
        this.rotorInertiaKgM2 = rotorInertiaKgM2;
        this.mass = mass;
    }
    
    /**
     * Calculates torque at a given RPM using the linear motor model.
     * T = T_stall * (1 - rpm / free_speed)
     */
    public double getTorqueAtRpm(double rpm) {
        if (rpm >= freeSpeedRpm) {
            return 0;
        }
        return stallTorqueNm * (1.0 - rpm / freeSpeedRpm);
    }
    
    /**
     * Calculates current draw at a given RPM.
     */
    public double getCurrentAtRpm(double rpm) {
        if (rpm >= freeSpeedRpm) {
            return freeCurrentAmps;
        }
        double torqueRatio = getTorqueAtRpm(rpm) / stallTorqueNm;
        return freeCurrentAmps + (stallCurrentAmps - freeCurrentAmps) * torqueRatio;
    }
    
    /**
     * Calculates mechanical power output at a given RPM.
     * P = T * omega
     */
    public double getPowerAtRpm(double rpm) {
        double omega = rpm * Math.PI / 30.0; // Convert to rad/s
        return getTorqueAtRpm(rpm) * omega;
    }
    
    /**
     * Calculates the RPM at which maximum power occurs.
     * For a linear motor model, this is at 50% of free speed.
     */
    public double getMaxPowerRpm() {
        return freeSpeedRpm / 2.0;
    }
    
    /**
     * Calculates maximum mechanical power output.
     */
    public double getMaxPower() {
        return getPowerAtRpm(getMaxPowerRpm());
    }
    
    /**
     * Calculates the Kv constant (RPM per volt).
     */
    public double getKv() {
        return freeSpeedRpm / nominalVoltage;
    }
    
    /**
     * Calculates the Kt constant (torque per amp).
     */
    public double getKt() {
        return stallTorqueNm / stallCurrentAmps;
    }
    
    /**
     * Calculates motor resistance in ohms.
     */
    public double getResistance() {
        return nominalVoltage / stallCurrentAmps;
    }
    
    /**
     * Estimates time to reach target RPM from rest, given flywheel inertia.
     * Uses a simplified linear acceleration model.
     * 
     * @param targetRpm Target RPM
     * @param flywheelInertia Total flywheel inertia in kg*m^2
     * @return Estimated time to reach target RPM in seconds
     */
    public double estimateSpinUpTime(double targetRpm, double flywheelInertia) {
        // Average torque during spin-up (midpoint approximation)
        double avgTorque = getTorqueAtRpm(targetRpm / 2.0);
        double totalInertia = flywheelInertia + rotorInertiaKgM2;
        
        // omega = alpha * t, where alpha = T / I
        double targetOmega = targetRpm * Math.PI / 30.0;
        double alpha = avgTorque / totalInertia;
        
        return targetOmega / alpha;
    }
    
    // Getters
    public String getName() { return name; }
    public double getFreeSpeedRpm() { return freeSpeedRpm; }
    public double getStallTorqueNm() { return stallTorqueNm; }
    public double getFreeCurrentAmps() { return freeCurrentAmps; }
    public double getStallCurrentAmps() { return stallCurrentAmps; }
    public double getNominalVoltage() { return nominalVoltage; }
    public double getRotorInertiaKgM2() { return rotorInertiaKgM2; }
    public double getMass() { return mass; }
    
    @Override
    public String toString() {
        return String.format("%s (%.0f RPM free, %.2f Nm stall)", name, freeSpeedRpm, stallTorqueNm);
    }
}
