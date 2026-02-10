package ca.team4308.absolutelib.subsystems.simulation;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

/** Simulation for a multi-jointed arm mechanism using WPILib physics. */
public class ArmSimulation extends SimulationBase {

    /** Configuration for a single simulated joint. */
    public static class JointSimConfig {

        /** Motor type for this joint. */
        public DCMotor gearbox = DCMotor.getNEO(1);
        /** Gear reduction ratio (motor turns per output turn). */
        public double gearRatio = 1.0;
        /** Length of this link in meters. */
        public double linkLengthMeters = 0.5;
        /** Mass of this link in kilograms. */
        public double linkMassKg = 3.0;
        /** Minimum angle limit in radians. */
        public double minAngleRad = -Math.PI;
        /** Maximum angle limit in radians. */
        public double maxAngleRad = Math.PI;
        /** Whether gravity should be simulated for this joint. */
        public boolean simulateGravity = true;
        /** Starting angle in radians. */
        public double startAngleRad = 0.0;
        /** Damping coefficient in Nm per rad/s. */
        public double dampingNmPerRadPerSec = 0.0;

        /**
         * Sets the motor type.
         *
         * @param motor the DC motor model
         * @return this config
         */
        public JointSimConfig gearbox(DCMotor motor) {
            this.gearbox = motor;
            return this;
        }

        /**
         * Sets the gear ratio.
         *
         * @param ratio gear reduction ratio
         * @return this config
         */
        public JointSimConfig gearRatio(double ratio) {
            this.gearRatio = ratio;
            return this;
        }

        /**
         * Sets the link length.
         *
         * @param meters link length in meters
         * @return this config
         */
        public JointSimConfig linkLength(double meters) {
            this.linkLengthMeters = meters;
            return this;
        }

        /**
         * Sets the link mass.
         *
         * @param kg link mass in kilograms
         * @return this config
         */
        public JointSimConfig linkMass(double kg) {
            this.linkMassKg = kg;
            return this;
        }

        /**
         * Sets the angle limits.
         *
         * @param minRad minimum angle in radians
         * @param maxRad maximum angle in radians
         * @return this config
         */
        public JointSimConfig limits(double minRad, double maxRad) {
            this.minAngleRad = minRad;
            this.maxAngleRad = maxRad;
            return this;
        }

        /**
         * Sets the starting angle.
         *
         * @param rad start angle in radians
         * @return this config
         */
        public JointSimConfig startAngle(double rad) {
            this.startAngleRad = rad;
            return this;
        }

        /**
         * Enables or disables gravity simulation.
         *
         * @param enable true to simulate gravity
         * @return this config
         */
        public JointSimConfig gravity(boolean enable) {
            this.simulateGravity = enable;
            return this;
        }

        /**
         * Sets the damping coefficient.
         *
         * @param nmPerRadPerSec damping in Nm per rad/s
         * @return this config
         */
        public JointSimConfig damping(double nmPerRadPerSec) {
            this.dampingNmPerRadPerSec = nmPerRadPerSec;
            return this;
        }
    }

    /** Aggregate configuration for the arm simulation. */
    public static class Config {

        /** The joint configurations for this arm. */
        public List<JointSimConfig> joints = new ArrayList<>();

        /**
         * Adds a joint to this arm configuration.
         *
         * @param joint the joint configuration to add
         * @return this config
         */
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
    private final ca.team4308.absolutelib.subsystems.Arm realArm;

    private final Mechanism2d mech2d = new Mechanism2d(3.0, 3.0);
    private final MechanismRoot2d armRoot = mech2d.getRoot("ArmBase", 1.5, 0.5);
    private final List<MechanismLigament2d> jointLigaments = new ArrayList<>();

    /**
     * Creates a named arm simulation.
     *
     * @param name    simulation name for telemetry
     * @param config  arm simulation configuration
     * @param realArm the real arm subsystem to synchronize with
     */
    public ArmSimulation(String name, Config config, ca.team4308.absolutelib.subsystems.Arm realArm) {
        super(name);
        this.config = config;
        this.realArm = realArm;

        for (JointSimConfig jcfg : config.joints) {
            joints.add(new JointSim(jcfg));
        }

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

    /**
     * Creates an arm simulation with the default name.
     *
     * @param realArm the real arm subsystem
     * @param config  arm simulation configuration
     */
    public ArmSimulation(ca.team4308.absolutelib.subsystems.Arm realArm, Config config) {
        this("arm", config, realArm);
    }

    /**
     * Public update method called by subsystems.
     *
     * @param dtSeconds time step in seconds
     */
    public void simUpdate(double dtSeconds) {
        super.update(dtSeconds);
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

        if (realArm != null) {
            List<ca.team4308.absolutelib.subsystems.Arm.Joint> realJoints = realArm.getJoints();
            int n = Math.min(joints.size(), realJoints.size());

            for (int i = 0; i < n; i++) {
                JointSim simJoint = joints.get(i);
                ca.team4308.absolutelib.subsystems.Arm.Joint realJoint = realJoints.get(i);

                double volts = realJoint.getMotor().getAppliedVoltage();
                simJoint.appliedVoltage = volts;

                double clamped = clamp(simJoint.appliedVoltage, -12.0, 12.0);
                simJoint.sim.setInputVoltage(clamped);
                simJoint.sim.update(dtSeconds);

                double angleRad = simJoint.sim.getAngleRads();
                double angleRot = angleRad / (2.0 * Math.PI);
                realJoint.getEncoder().setSimulatedPositionMechanismRotations(angleRot);

                double velocityRotPerSec = simJoint.sim.getVelocityRadPerSec() / (2.0 * Math.PI);

                double rotorRot = angleRot * simJoint.config.gearRatio;
                double rotorVel = velocityRotPerSec * simJoint.config.gearRatio;

                realJoint.getMotor().updateSimState(rotorRot, rotorVel);

                totalCurrent += simJoint.sim.getCurrentDrawAmps();
            }
        } else {
            for (JointSim j : joints) {
                double clamped = clamp(j.appliedVoltage, -12.0, 12.0);
                j.sim.setInputVoltage(clamped);
                j.sim.update(dtSeconds);
                totalCurrent += j.sim.getCurrentDrawAmps();
            }
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

            recordSimOutput("joint" + i + "/angleDeg", angles[i]);
            recordSimOutput("joint" + i + "/velocityDegPerSec", velocities[i]);
            recordSimOutput("joint" + i + "/currentAmps", currents[i]);
            recordSimOutput("joint" + i + "/hitLowerLimit", j.sim.hasHitLowerLimit());
            recordSimOutput("joint" + i + "/hitUpperLimit", j.sim.hasHitUpperLimit());
        }

        for (int i = 0; i < Math.min(jointLigaments.size(), joints.size()); i++) {
            jointLigaments.get(i).setAngle(angles[i]);
        }

        recordSimOutput("jointAngles", toRadians(angles));

        double[] fk = computeForwardKinematics();
        recordSimOutput("endEffector", new Pose2d(fk[0], fk[1], new Rotation2d(fk[2])));

        recordSimOutput("mechanism2d", mech2d);

        if (!joints.isEmpty()) {
            recordSimOutput("3d/pose", new Pose3d(0, 0, 1.0, new Rotation3d(0, -joints.get(0).sim.getAngleRads(), 0)));
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

    /**
     * Sets the applied voltage for a specific joint.
     *
     * @param index joint index
     * @param volts voltage to apply
     */
    public void setJointVoltage(int index, double volts) {
        if (index >= 0 && index < joints.size()) {
            joints.get(index).appliedVoltage = volts;
        }
    }

    /**
     * Sets the applied voltages for all joints.
     *
     * @param volts voltages to apply, one per joint
     */
    public void setJointVoltages(double... volts) {
        for (int i = 0; i < Math.min(volts.length, joints.size()); i++) {
            joints.get(i).appliedVoltage = volts[i];
        }
    }

    /**
     * Returns the angle of a joint in radians.
     *
     * @param index joint index
     * @return angle in radians, or 0 if index is out of range
     */
    public double getJointAngleRad(int index) {
        return (index >= 0 && index < joints.size()) ? joints.get(index).sim.getAngleRads() : 0.0;
    }

    /**
     * Returns the angle of a joint in degrees.
     *
     * @param index joint index
     * @return angle in degrees
     */
    public double getJointAngleDeg(int index) {
        return Math.toDegrees(getJointAngleRad(index));
    }

    /**
     * Returns the angular velocity of a joint in rad/s.
     *
     * @param index joint index
     * @return velocity in rad/s, or 0 if index is out of range
     */
    public double getJointVelocityRadPerSec(int index) {
        return (index >= 0 && index < joints.size()) ? joints.get(index).sim.getVelocityRadPerSec() : 0.0;
    }

    /**
     * Returns the angular velocity of a joint in deg/s.
     *
     * @param index joint index
     * @return velocity in deg/s
     */
    public double getJointVelocityDegPerSec(int index) {
        return Math.toDegrees(getJointVelocityRadPerSec(index));
    }

    /**
     * Sets the simulated angle of a joint.
     *
     * @param index    joint index
     * @param angleRad angle in radians
     */
    public void setJointAngle(int index, double angleRad) {
        if (index >= 0 && index < joints.size()) {
            JointSim j = joints.get(index);
            j.sim.setState(angleRad, j.sim.getVelocityRadPerSec());
        }
    }

    /**
     * Returns the number of joints in this simulation.
     *
     * @return joint count
     */
    public int getJointCount() {
        return joints.size();
    }

    /** Resets all joints to their starting angles with zero velocity. */
    public void reset() {
        for (JointSim j : joints) {
            j.sim.setState(j.config.startAngleRad, 0.0);
            j.appliedVoltage = 0.0;
        }
        logInfo("Arm simulation reset");
    }

    /**
     * Returns the WPILib Mechanism2d visualization object.
     *
     * @return the mechanism 2d widget
     */
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
