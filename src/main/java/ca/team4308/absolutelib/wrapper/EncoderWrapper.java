package ca.team4308.absolutelib.wrapper;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/**
 * EncoderWrapper provides a minimal, vendor-agnostic interface for fetching and
 * setting position. Supports both linear (meters) and rotational
 * (rotations/radians) units.
 */
public interface EncoderWrapper {

    /**
     * Current linear position in meters. Requires drumDiameter to be
     * configured.
     */
    double getPositionMeters();

    /**
     * Reset/report the linear position in meters.
     */
    void setPositionMeters(double meters);

    /**
     * Current position in mechanism rotations.
     */
    double getPositionMechanismRotations();

    /**
     * Reset/report position in mechanism rotations.
     */
    void setPositionMechanismRotations(double rotations);

    /**
     * Update the simulated sensor state (if applicable). Used to feed
     * simulation physics back into the sensor so control loops work in sim.
     */
    default void setSimulatedPositionMechanismRotations(double rotations) {
    }

    /**
     * Current position in radians.
     */
    default double getPositionRadians() {
        return getPositionMechanismRotations() * 2.0 * Math.PI;
    }

    /**
     * Reset/report position in radians.
     */
    default void setPositionRadians(double rads) {
        setPositionMechanismRotations(rads / (2.0 * Math.PI));
    }

    /**
     * Returns true if this encoder is absolute (position persists across power
     * cycles).
     */
    default boolean isAbsolute() {
        return false;
    }

    // --- Factories ---
    static EncoderWrapper canCoder(int canId, double gearRatio, double countsPerRev, double drumDiameter) {
        return new CANCoderImpl(canId, gearRatio, countsPerRev, drumDiameter);
    }

    static EncoderWrapper canCoder(int canId, double gearRatio) {
        return new CANCoderImpl(canId, gearRatio, 4096.0, 0.0);
    }

    static EncoderWrapper talonFXIntegrated(int canId, double gearRatio, double drumDiameter) {
        return talonFXIntegrated(canId, gearRatio, 2048.0, drumDiameter);
    }

    static EncoderWrapper talonFXIntegrated(int canId, double gearRatio) {
        return new TalonFXIntegratedImpl(canId, gearRatio, 2048.0, 0.0);
    }

    static EncoderWrapper talonFXIntegrated(int canId, double gearRatio, double countsPerRev, double drumDiameter) {
        return new TalonFXIntegratedImpl(canId, gearRatio, countsPerRev, drumDiameter);
    }

    static EncoderWrapper dioEncoder(int channelA, int channelB, boolean reversed,
            double gearRatio, double countsPerRev, double drumDiameter) {
        return new DioEncoderImpl(channelA, channelB, reversed, gearRatio, countsPerRev, drumDiameter);
    }

    static EncoderWrapper dutyCycleEncoder(int channel, double gearRatio, double drumDiameter) {
        return new DutyCycleEncoderImpl(channel, gearRatio, drumDiameter);
    }

    static EncoderWrapper dutyCycleEncoder(int channel, double gearRatio) {
        return new DutyCycleEncoderImpl(channel, gearRatio, 0.0);
    }

    static EncoderWrapper ofEncoderCounts(Supplier<Double> getCounts, DoubleConsumer setCounts,
            double gearRatio, double countsPerRev, double drumDiameter) {
        return new GenericCountsImpl(getCounts, setCounts, null, gearRatio, countsPerRev, drumDiameter);
    }

    static EncoderWrapper ofEncoderCounts(Supplier<Double> getCounts, DoubleConsumer setCounts, DoubleConsumer setSimCounts,
            double gearRatio, double countsPerRev, double drumDiameter) {
        return new GenericCountsImpl(getCounts, setCounts, setSimCounts, gearRatio, countsPerRev, drumDiameter);
    }

    static EncoderWrapper ofMechanismRotations(Supplier<Double> getMechanismRotations,
            DoubleConsumer setMechanismRotations,
            double drumDiameter) {
        return new GenericMechanismRotationsImpl(getMechanismRotations, setMechanismRotations, null, drumDiameter);
    }

    static EncoderWrapper ofMechanismRotations(Supplier<Double> getMechanismRotations,
            DoubleConsumer setMechanismRotations,
            DoubleConsumer setSimMechanismRotations,
            double drumDiameter) {
        return new GenericMechanismRotationsImpl(getMechanismRotations, setMechanismRotations, setSimMechanismRotations, drumDiameter);
    }

    static EncoderWrapper ofAbsoluteRotations0To1(Supplier<Double> getSensorRotations,
            double gearRatio,
            double drumDiameter) {
        return new GenericAbsoluteRotationsImpl(getSensorRotations, null, gearRatio, drumDiameter);
    }

    static EncoderWrapper ofAbsoluteRotations0To1(Supplier<Double> getSensorRotations,
            DoubleConsumer setSimSensorRotations,
            double gearRatio,
            double drumDiameter) {
        return new GenericAbsoluteRotationsImpl(getSensorRotations, setSimSensorRotations, gearRatio, drumDiameter);
    }

    static EncoderWrapper ofAbsoluteRotations0To1(Supplier<Double> getSensorRotations, double gearRatio) {
        return new GenericAbsoluteRotationsImpl(getSensorRotations, null, gearRatio, 0.0);
    }

    // --- Implementations ---
    class CANCoderImpl implements EncoderWrapper {

        private final CANcoder encoder;
        private final double gearRatio;
        private final double drumDiameter;

        public CANCoderImpl(int canId, double gearRatio, double countsPerRev, double drumDiameter) {
            this.encoder = new CANcoder(canId);
            this.gearRatio = gearRatio;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public boolean isAbsolute() {
            return true;
        }

        @Override
        public double getPositionMechanismRotations() {
            return encoder.getPosition().getValueAsDouble() / gearRatio;
        }

        @Override
        public void setPositionMechanismRotations(double rotations) {
            encoder.setPosition(rotations * gearRatio);
        }

        @Override
        public void setSimulatedPositionMechanismRotations(double rotations) {
            encoder.getSimState().setRawPosition(rotations * gearRatio);
        }

        @Override
        public double getPositionMeters() {
            return getPositionMechanismRotations() * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            if (drumDiameter != 0) {
                setPositionMechanismRotations(meters / (Math.PI * drumDiameter));
            }
        }
    }

    class TalonFXIntegratedImpl implements EncoderWrapper {

        private final TalonFX talon;
        private final double gearRatio;
        private final double drumDiameter;

        public TalonFXIntegratedImpl(int canId, double gearRatio, double countsPerRev, double drumDiameter) {
            this.talon = new TalonFX(canId);
            this.gearRatio = gearRatio;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public boolean isAbsolute() {
            return false;
        }

        @Override
        public double getPositionMechanismRotations() {
            return talon.getPosition().getValueAsDouble() / gearRatio;
        }

        @Override
        public void setPositionMechanismRotations(double rotations) {
            talon.setPosition(rotations * gearRatio);
        }

        @Override
        public void setSimulatedPositionMechanismRotations(double rotations) {
            talon.getSimState().setRawRotorPosition(rotations * gearRatio);
        }

        @Override
        public double getPositionMeters() {
            return getPositionMechanismRotations() * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            if (drumDiameter != 0) {
                setPositionMechanismRotations(meters / (Math.PI * drumDiameter));
            }
        }
    }

    class DioEncoderImpl implements EncoderWrapper {

        private final Encoder encoder;
        private final EncoderSim sim;
        private final double drumDiameter;
        private final double gearRatio;
        private final double countsPerRev;
        private double offsetRotations = 0.0;

        public DioEncoderImpl(int channelA, int channelB, boolean reversed,
                double gearRatio, double countsPerRev, double drumDiameter) {
            this.encoder = new Encoder(channelA, channelB, reversed);
            this.sim = new EncoderSim(encoder);
            this.drumDiameter = drumDiameter;
            this.gearRatio = gearRatio;
            this.countsPerRev = countsPerRev;
            this.encoder.setDistancePerPulse(1.0);
        }

        @Override
        public boolean isAbsolute() {
            return false;
        }

        @Override
        public double getPositionMechanismRotations() {
            double ticks = encoder.getDistance();
            double sensorRot = ticks / countsPerRev;
            return offsetRotations + (sensorRot / gearRatio);
        }

        @Override
        public void setPositionMechanismRotations(double rotations) {
            encoder.reset();
            offsetRotations = rotations;
        }

        @Override
        public void setSimulatedPositionMechanismRotations(double rotations) {
            // Back-calculate distance (ticks)
            double sensorRot = (rotations - offsetRotations) * gearRatio;
            double ticks = sensorRot * countsPerRev;
            sim.setDistance(ticks);
        }

        @Override
        public double getPositionMeters() {
            return getPositionMechanismRotations() * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            if (drumDiameter != 0) {
                setPositionMechanismRotations(meters / (Math.PI * drumDiameter));
            }
        }
    }

    class DutyCycleEncoderImpl implements EncoderWrapper {

        private final DutyCycleEncoder encoder;
        private final DutyCycleEncoderSim sim;
        private final double gearRatio;
        private final double drumDiameter;
        private double offsetRotations = 0.0;

        public DutyCycleEncoderImpl(int channel, double gearRatio, double drumDiameter) {
            this.encoder = new DutyCycleEncoder(channel);
            this.sim = new DutyCycleEncoderSim(encoder);
            this.gearRatio = gearRatio;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public boolean isAbsolute() {
            return true;
        }

        @Override
        public double getPositionMechanismRotations() {
            double sensorRot = encoder.get();
            double mechRot = sensorRot / gearRatio;
            return mechRot + offsetRotations;
        }

        @Override
        public void setPositionMechanismRotations(double rotations) {
            offsetRotations = rotations - (encoder.get() / gearRatio);
        }

        @Override
        public void setSimulatedPositionMechanismRotations(double rotations) {
            // Set raw sensor value (0-1 usually, but can wrap)
            double sensorRot = (rotations - offsetRotations) * gearRatio;
            sim.set(sensorRot);
        }

        @Override
        public double getPositionMeters() {
            return getPositionMechanismRotations() * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            if (drumDiameter != 0) {
                setPositionMechanismRotations(meters / (Math.PI * drumDiameter));
            }
        }
    }

    class GenericCountsImpl implements EncoderWrapper {

        private final Supplier<Double> countsSupplier;
        private final DoubleConsumer countsSetter;
        private final DoubleConsumer simCountsSetter;
        private final double gearRatio;
        private final double countsPerRev;
        private final double drumDiameter;

        public GenericCountsImpl(Supplier<Double> getCounts, DoubleConsumer setCounts, DoubleConsumer setSimCounts,
                double gearRatio, double countsPerRev, double drumDiameter) {
            this.countsSupplier = getCounts;
            this.countsSetter = setCounts;
            this.simCountsSetter = setSimCounts;
            this.gearRatio = gearRatio;
            this.countsPerRev = countsPerRev;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public boolean isAbsolute() {
            return false;
        }

        @Override
        public double getPositionMechanismRotations() {
            return (countsSupplier.get() / countsPerRev) / gearRatio;
        }

        @Override
        public void setPositionMechanismRotations(double rotations) {
            countsSetter.accept(rotations * gearRatio * countsPerRev);
        }

        @Override
        public double getPositionMeters() {
            return getPositionMechanismRotations() * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            if (drumDiameter != 0) {
                setPositionMechanismRotations(meters / (Math.PI * drumDiameter));
            }
        }

        @Override
        public void setSimulatedPositionMechanismRotations(double rotations) {
            if (simCountsSetter != null) {
                simCountsSetter.accept(rotations * gearRatio * countsPerRev);
            }
        }
    }

    class GenericMechanismRotationsImpl implements EncoderWrapper {

        private final Supplier<Double> rotationsSupplier;
        private final DoubleConsumer rotationsSetter;
        private final DoubleConsumer simRotationsSetter;
        private final double drumDiameter;

        public GenericMechanismRotationsImpl(Supplier<Double> rotationsSupplier,
                DoubleConsumer rotationsSetter,
                DoubleConsumer simRotationsSetter,
                double drumDiameter) {
            this.rotationsSupplier = rotationsSupplier;
            this.rotationsSetter = rotationsSetter;
            this.simRotationsSetter = simRotationsSetter;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public double getPositionMechanismRotations() {
            return rotationsSupplier.get();
        }

        @Override
        public void setPositionMechanismRotations(double rotations) {
            rotationsSetter.accept(rotations);
        }

        @Override
        public double getPositionMeters() {
            return getPositionMechanismRotations() * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            if (drumDiameter != 0) {
                setPositionMechanismRotations(meters / (Math.PI * drumDiameter));
            }
        }

        @Override
        public void setSimulatedPositionMechanismRotations(double rotations) {
            if (simRotationsSetter != null) {
                simRotationsSetter.accept(rotations);
            }
        }
    }

    class GenericAbsoluteRotationsImpl implements EncoderWrapper {

        private final Supplier<Double> absoluteRotationsSupplier;
        private final DoubleConsumer simRotationsSetter;
        private final double gearRatio;
        private final double drumDiameter;
        private double offsetRotations = 0.0;

        public GenericAbsoluteRotationsImpl(Supplier<Double> absoluteRotationsSupplier,
                DoubleConsumer simRotationsSetter,
                double gearRatio,
                double drumDiameter) {
            this.absoluteRotationsSupplier = absoluteRotationsSupplier;
            this.simRotationsSetter = simRotationsSetter;
            this.gearRatio = gearRatio;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public boolean isAbsolute() {
            return true;
        }

        @Override
        public double getPositionMechanismRotations() {
            return (absoluteRotationsSupplier.get() / gearRatio) + offsetRotations;
        }

        @Override
        public void setPositionMechanismRotations(double rotations) {
            offsetRotations = rotations - (absoluteRotationsSupplier.get() / gearRatio);
        }

        @Override
        public double getPositionMeters() {
            return getPositionMechanismRotations() * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            if (drumDiameter != 0) {
                setPositionMechanismRotations(meters / (Math.PI * drumDiameter));
            }
        }

        @Override
        public void setSimulatedPositionMechanismRotations(double rotations) {
            if (simRotationsSetter != null) {
                simRotationsSetter.accept(rotations * gearRatio);
            }
        }
    }
}
