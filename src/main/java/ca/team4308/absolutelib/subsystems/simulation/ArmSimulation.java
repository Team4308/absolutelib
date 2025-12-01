package ca.team4308.absolutelib.subsystems.simulation;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

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
        public double dampingNmPerRadPerSec = 0.0;

        public JointSimConfig gearbox(DCMotor motor) { this.gearbox = motor; return this; }
        public JointSimConfig gearRatio(double ratio) { this.gearRatio = ratio; return this; }
        public JointSimConfig linkLength(double meters) { this.linkLengthMeters = meters; return this; }
        public JointSimConfig linkMass(double kg) { this.linkMassKg = kg; return this; }
        public JointSimConfig limits(double minRad, double maxRad) { this.minAngleRad = minRad; this.maxAngleRad = maxRad; return this; }
        public JointSimConfig startAngle(double rad) { this.startAngleRad = rad; return this; }
        public JointSimConfig gravity(boolean enable) { this.simulateGravity = enable; return this; }
        public JointSimConfig damping(double nmPerRadPerSec) { this.dampingNmPerRadPerSec = nmPerRadPerSec; return this; }
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
            double moi = (1.0 / 3.0) * cfg.linkMassKg * cfg.linkLengthMeters * cfg.linkLengthMeters;
            this.sim = new SingleJointedArmSim(
                cfg.gearbox,
                cfg.gearRatio,
                moi,
                cfg.linkLengthMeters,
                cfg.minAngleRad,
                cfg.maxAngleRad,
                cfg.simulateGravity,
                cfg.startAngleRad
            );
        }
    }

    private final List<JointSim> joints = new ArrayList<>();
    private final Config config;
    private final SimState currentState = new SimState();
    private final String name;

    // Mechanism2d visualization fields
    private final Mechanism2d mech2d = new Mechanism2d(3.0, 3.0);
    private final MechanismRoot2d armRoot = mech2d.getRoot("ArmBase", 1.5, 0.5);
    private final List<MechanismLigament2d> jointLigaments = new ArrayList<>();

    public ArmSimulation(String name, Config config) {
        super(name);
        this.name = name;
        this.config = config;
        for (JointSimConfig jcfg : config.joints) {
            joints.add(new JointSim(jcfg));
        }

        // Build a chain of ligaments, one per joint
        MechanismLigament2d parent = null;
        for (int i = 0; i < joints.size(); i++) {
            JointSimConfig jcfg = joints.get(i).config;
            MechanismLigament2d lig = new MechanismLigament2d(
                "joint" + i,
                jcfg.linkLengthMeters,
                Math.toDegrees(jcfg.startAngleRad)
            );
            if (parent == null) {
                armRoot.append(lig);
            } else {
                parent.append(lig);
            }
            jointLigaments.add(lig);
            parent = lig;
        }
    }

    public ArmSimulation(Config config) {
        this("arm", config);
    }

    /**
     * Public update method called by subsystems.
     */
    public void simUpdate(double dtSeconds) {
        updateSimulation(dtSeconds);
        onSimulationPeriodic(dtSeconds);
    }

    @Override
    protected void onSimulationInit() {
        for (JointSim j : joints) {
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

            recordOutput(name + "/joint" + i + "/angleDeg", angles[i]);
            recordOutput(name + "/joint" + i + "/velocityDegPerSec", velocities[i]);
            recordOutput(name + "/joint" + i + "/currentAmps", currents[i]);
            recordOutput(name + "/joint" + i + "/hitLowerLimit", j.sim.hasHitLowerLimit());
            recordOutput(name + "/joint" + i + "/hitUpperLimit", j.sim.hasHitUpperLimit());
        }

        // Update Mechanism2d joint ligaments
        for (int i = 0; i < Math.min(jointLigaments.size(), joints.size()); i++) {
            jointLigaments.get(i).setAngle(angles[i]);
        }

        recordOutput(name + "/jointAngles", toRadians(angles));

        // Forward kinematics for end effector
        double[] fk = computeForwardKinematics();
        recordOutput(name + "/endEffector", new Pose2d(fk[0], fk[1], new Rotation2d(fk[2])));

        // Log mechanism2d data using built-in recordOutput
        recordOutput(name + "/mechanism2d", mech2d);

        // 3D Visualization (Simple single joint approximation)
        if (!joints.isEmpty()) {
            recordOutput(name + "/3d/pose", new Pose3d(0, 0, 1.0, new Rotation3d(0, -joints.get(0).sim.getAngleRads(), 0)));
        }
    }

    private double[] toRadians(double[] degrees) {
        double[] rad = new double[degrees.length];
        for (int i = 0; i < degrees.length; i++) {
            rad[i] = Math.toRadians(degrees[i]);
        }
        return rad;
    }

    private double[] computeForwardKinematics() {
        if (joints.isEmpty()) {
            return new double[]{0, 0, 0};
        }

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

    public double getJointVelocityRadPerSec(int index) {
        return (index >= 0 && index < joints.size()) ? joints.get(index).sim.getVelocityRadPerSec() : 0.0;
    }

    public double getJointVelocityDegPerSec(int index) {
        return Math.toDegrees(getJointVelocityRadPerSec(index));
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

    public Mechanism2d getMechanism2d() {
        return mech2d;
    }

    @Override
    protected void onStop() {
        for (JointSim j : joints) {
            j.sim.setState(j.config.startAngleRad, 0.0);
            j.appliedVoltage = 0.0;
        }
    }
}
