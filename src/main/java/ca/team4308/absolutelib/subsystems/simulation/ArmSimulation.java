package ca.team4308.absolutelib.subsystems.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;

public class ArmSimulation extends SimulationBase {

    public static class JointSimConfig {
        public DCMotor gearbox = DCMotor.getNEO(1);
        public double gearRatio = 1.0;
        public double linkLengthMeters = 0.5;
        public double linkMassKg = 3.0;
        public double minAngleRad = -Math.PI;
        public double maxAngleRad = Math.PI;
        public boolean simulateGravity = true;
        public double startAngleRad = 0.0;

        public JointSimConfig gearbox(DCMotor motor) { this.gearbox = motor; return this; }
        public JointSimConfig gearRatio(double ratio) { this.gearRatio = ratio; return this; }
        public JointSimConfig linkLength(double meters) { this.linkLengthMeters = meters; return this; }
        public JointSimConfig linkMass(double kg) { this.linkMassKg = kg; return this; }
        public JointSimConfig limits(double minRad, double maxRad) { 
            this.minAngleRad = minRad; this.maxAngleRad = maxRad; return this; 
        }
        public JointSimConfig startAngle(double rad) { this.startAngleRad = rad; return this; }
        public JointSimConfig gravity(boolean enable) { this.simulateGravity = enable; return this; }
    }

    public static class Config {
        public List<JointSimConfig> joints = new ArrayList<>();
        
        public Config addJoint(JointSimConfig joint) {
            joints.add(joint);
            return this;
        }
    }

    private static class JointSim {
        final SingleJointedArmSim sim;
        final JointSimConfig config;
        double appliedVoltage = 0.0;

        JointSim(JointSimConfig cfg) {
            this.config = cfg;
            double moi = (1.0/3.0) * cfg.linkMassKg * cfg.linkLengthMeters * cfg.linkLengthMeters;
            this.sim = new SingleJointedArmSim(
                cfg.gearbox, cfg.gearRatio, moi, cfg.linkLengthMeters,
                cfg.minAngleRad, cfg.maxAngleRad, cfg.simulateGravity, cfg.startAngleRad
            );
        }
    }

    private final List<JointSim> joints = new ArrayList<>();
    private final Config config;
    private final SimState currentState = new SimState();

    public ArmSimulation(String name, Config config) {
        super(name);
        this.config = config;
        for (JointSimConfig jcfg : config.joints) {
            joints.add(new JointSim(jcfg));
        }
    }

    public ArmSimulation(Config config) {
        this("arm", config);
    }

    @Override
    protected void onSimulationInit() {
        for (int i = 0; i < joints.size(); i++) {
            JointSim j = joints.get(i);
            j.sim.setState(j.config.startAngleRad, 0.0);
        }
        logInfo("Arm simulation initialized with " + joints.size() + " joints");
    }

    @Override
    protected void updateSimulation(double dtSeconds) {
        double totalCurrent = 0.0;
        for (JointSim j : joints) {
            double clamped = clamp(j.appliedVoltage, -12.0, 12.0);
            j.sim.setInputVoltage(clamped);
            j.sim.update(dtSeconds);
            totalCurrent += j.sim.getCurrentDrawAmps();
        }
        
        if (!joints.isEmpty()) {
            JointSim first = joints.get(0);
            currentState.positionMeters = first.sim.getAngleRads();
            currentState.velocityMetersPerSec = first.sim.getVelocityRadPerSec();
            currentState.appliedVoltage = first.appliedVoltage;
        }
        currentState.currentDrawAmps = totalCurrent;
        currentState.temperatureCelsius = 20.0 + (totalCurrent * 1.5);
    }

    @Override
    protected SimState getSimulationState() {
        return currentState;
    }

    @Override
    protected void onSimulationPeriodic(double dtSeconds) {
        // Log each joint
        double[] angles = new double[joints.size()];
        double[] velocities = new double[joints.size()];
        double[] currents = new double[joints.size()];
        
        for (int i = 0; i < joints.size(); i++) {
            JointSim j = joints.get(i);
            angles[i] = Math.toDegrees(j.sim.getAngleRads());
            velocities[i] = Math.toDegrees(j.sim.getVelocityRadPerSec());
            currents[i] = j.sim.getCurrentDrawAmps();
            
            SDAdd("joint" + i + "/angleDeg", angles[i]);
            SDAdd("joint" + i + "/velocityDegPerSec", velocities[i]);
            SDAdd("joint" + i + "/currentAmps", currents[i]);
            SDAdd("joint" + i + "/hitLowerLimit", j.sim.hasHitLowerLimit());
            SDAdd("joint" + i + "/hitUpperLimit", j.sim.hasHitUpperLimit());
        }
        
        logJointAngles(toRadians(angles));
        
        // Forward kinematics for end effector
        double[] fk = computeForwardKinematics();
        logPose2d("endEffector", fk[0], fk[1], fk[2]);
        
        // Log mechanism2d data
        Logger.recordOutput(getLogChannelBase() + "/mechanism2d/angles", angles);
    }

    private double[] toRadians(double[] degrees) {
        double[] rad = new double[degrees.length];
        for (int i = 0; i < degrees.length; i++) rad[i] = Math.toRadians(degrees[i]);
        return rad;
    }

    /** Compute end effector pose using forward kinematics */
    private double[] computeForwardKinematics() {
        if (joints.isEmpty()) return new double[]{0, 0, 0};
        
        double x = 0, y = 0, rotation = 0;
        for (JointSim j : joints) {
            double angle = j.sim.getAngleRads() + rotation;
            x += j.config.linkLengthMeters * Math.cos(angle);
            y += j.config.linkLengthMeters * Math.sin(angle);
            rotation = angle;
        }
        return new double[]{x, y, rotation};
    }

    // Control methods
    public void setJointVoltage(int index, double volts) {
        if (index >= 0 && index < joints.size()) {
            joints.get(index).appliedVoltage = volts;
        }
    }

    public void setJointVoltages(double... volts) {
        for (int i = 0; i < Math.min(volts.length, joints.size()); i++) {
            joints.get(i).appliedVoltage = volts[i];
        }
    }

    public double getJointAngleRad(int index) {
        return (index >= 0 && index < joints.size()) ? joints.get(index).sim.getAngleRads() : 0.0;
    }

    public double getJointAngleDeg(int index) {
        return Math.toDegrees(getJointAngleRad(index));
    }

    public void setJointAngle(int index, double angleRad) {
        if (index >= 0 && index < joints.size()) {
            JointSim j = joints.get(index);
            j.sim.setState(angleRad, j.sim.getVelocityRadPerSec());
        }
    }

    public int getJointCount() {
        return joints.size();
    }

    public void reset() {
        for (JointSim j : joints) {
            j.sim.setState(j.config.startAngleRad, 0.0);
            j.appliedVoltage = 0.0;
        }
        logInfo("Arm simulation reset");
    }

    @Override
    protected void onStop() {
        for (JointSim j : joints) j.appliedVoltage = 0.0;
        super.onStop();
    }
}
