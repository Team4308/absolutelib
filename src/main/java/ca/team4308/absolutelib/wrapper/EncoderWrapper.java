package ca.team4308.absolutelib.wrapper;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.EncoderConversion;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

/**
 * EncoderWrapper provides a minimal, vendor-agnostic interface for fetching and setting
 * linear position (in meters). Use the static factory helpers to create wrappers for
 * common encoder types, or implement the interface yourself for custom sensors.
 */
public interface EncoderWrapper {
    /** Current linear position in meters. */
    double getPositionMeters();

    /** Reset/report the linear position in meters (if supported by the underlying sensor). */
    void setPositionMeters(double meters);

    /** Returns true if this encoder is absolute (position persists across power cycles). */
    default boolean isAbsolute() { return false; }

    /**
     * Create a CANCoder-backed encoder wrapper.
     * @param canId CAN ID of the CANCoder
     * @param gearRatio Sensor rotations per mechanism rotation
     * @param countsPerRev Native counts per encoder revolution (e.g. 4096 or 4028)
     * @param drumDiameter Mechanism drum diameter (meters) for linear conversion
     */
    static EncoderWrapper canCoder(int canId, double gearRatio, double countsPerRev, double drumDiameter) {
        return new CANCoderImpl(canId, gearRatio, countsPerRev, drumDiameter);
    }

    /** TalonFX integrated sensor (uses 2048 counts per rev by default). */
    static EncoderWrapper talonFXIntegrated(int canId, double gearRatio, double drumDiameter) {
        return talonFXIntegrated(canId, gearRatio, 2048.0, drumDiameter);
    }

    /** TalonFX integrated sensor with custom counts per rev (normally 2048). */
    static EncoderWrapper talonFXIntegrated(int canId, double gearRatio, double countsPerRev, double drumDiameter) {
        return new TalonFXIntegratedImpl(canId, gearRatio, countsPerRev, drumDiameter);
    }

    // Phoenix 5 SRX/VictorSPX quadrature not supported in Phoenix 6-only build.

    /** DIO-based WPILib Encoder (incremental). */
    static EncoderWrapper dioEncoder(int channelA, int channelB, boolean reversed,
                                     double gearRatio, double countsPerRev, double drumDiameter) {
        return new DioEncoderImpl(channelA, channelB, reversed, gearRatio, countsPerRev, drumDiameter);
    }

    /** DutyCycleEncoder absolute position (adds offset for setPositionMeters). */
    static EncoderWrapper dutyCycleEncoder(int channel, double gearRatio, double drumDiameter) {
        return new DutyCycleEncoderImpl(channel, gearRatio, drumDiameter);
    }

    /** Generic factory from raw encoder counts (ticks). */
    static EncoderWrapper ofEncoderCounts(Supplier<Double> getCounts, DoubleConsumer setCounts,
                                          double gearRatio, double countsPerRev, double drumDiameter) {
        return new GenericCountsImpl(getCounts, setCounts, gearRatio, countsPerRev, drumDiameter);
    }

    /** Generic factory from mechanism rotations directly. */
    static EncoderWrapper ofMechanismRotations(Supplier<Double> getMechanismRotations,
                                               DoubleConsumer setMechanismRotations,
                                               double drumDiameter) {
        return new GenericMechanismRotationsImpl(getMechanismRotations, setMechanismRotations, drumDiameter);
    }

    /**
     * Generic factory for absolute sensors that report position as sensor rotations in [0,1).
     * Maintains an internal offset to emulate setPositionMeters.
     */
    
    static EncoderWrapper ofAbsoluteRotations0To1(Supplier<Double> getSensorRotations,
                                                  double gearRatio,
                                                  double drumDiameter) {
        return new GenericAbsoluteRotationsImpl(getSensorRotations, gearRatio, drumDiameter);
    }

    /** Simple CANCoder implementation performing unit conversions to meters. */
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
        public boolean isAbsolute() { return true; }

        @Override
        public double getPositionMeters() {
            // Phoenix 6 CANcoder position is in rotations by default
            double sensorRot = encoder.getPosition().getValueAsDouble();
            double mechRot = sensorRot / gearRatio;
            return mechRot * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            double rotations = meters / (Math.PI * drumDiameter); // mechanism rotations
            double sensorRot = rotations * gearRatio;
            encoder.setPosition(sensorRot);
        }
    }

    /** TalonFX integrated sensor implementation. */
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
        public boolean isAbsolute() { return false; } // Relative encoder

        @Override
        public double getPositionMeters() {
            // Phoenix 6 TalonFX position is reported in rotations
            double sensorRot = talon.getPosition().getValueAsDouble();
            double mechRot = sensorRot / gearRatio;
            return mechRot * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            double rotations = meters / (Math.PI * drumDiameter); // mechanism rotations
            double sensorRot = rotations * gearRatio;
            talon.setPosition(sensorRot);
        }
    }

    // TalonSRX/VictorSPX implementation removed for Phoenix 6-only build

    /** WPILib DIO encoder implementation with offset for setPositionMeters. */
    class DioEncoderImpl implements EncoderWrapper {
        private final Encoder encoder;
        private final double drumDiameter;
        private double offsetMeters = 0.0;

        public DioEncoderImpl(int channelA, int channelB, boolean reversed,
                              double gearRatio, double countsPerRev, double drumDiameter) {
            this.encoder = new Encoder(channelA, channelB, reversed);
            this.drumDiameter = drumDiameter;
            // Compute meters per pulse using conversion utilities.
            double mechanismDegreesPerPulse = 360.0 / (countsPerRev * gearRatio);
            double metersPerPulse = EncoderConversion.degreesToMeters(mechanismDegreesPerPulse, drumDiameter);
            this.encoder.setDistancePerPulse(metersPerPulse);
        }

        @Override
        public boolean isAbsolute() { return false; } // Relative encoder

        @Override
        public double getPositionMeters() {
            return offsetMeters + encoder.getDistance();
        }

        @Override
        public void setPositionMeters(double meters) {
            encoder.reset();
            offsetMeters = meters;
        }
    }

    /** WPILib DutyCycleEncoder absolute implementation with offset for setPositionMeters. */
    class DutyCycleEncoderImpl implements EncoderWrapper {
        private final DutyCycleEncoder encoder;
        private final double gearRatio;
        private final double drumDiameter;
        private double offsetMeters = 0.0;

        public DutyCycleEncoderImpl(int channel, double gearRatio, double drumDiameter) {
            this.encoder = new DutyCycleEncoder(channel);
            this.gearRatio = gearRatio;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public boolean isAbsolute() { return true; }

        @Override
        public double getPositionMeters() {
            double sensorRot = encoder.get(); // 0-1 sensor rotations
            double mechanismRot = sensorRot / gearRatio;
            double meters = mechanismRot * Math.PI * drumDiameter;
            return meters + offsetMeters;
        }

        

        @Override
        public void setPositionMeters(double meters) {
            offsetMeters = meters - getPositionMeters();
        }
    }

    /** Generic counts-based implementation. */
    class GenericCountsImpl implements EncoderWrapper {
        private final Supplier<Double> countsSupplier;
        private final DoubleConsumer countsSetter;
        private final double gearRatio;
        private final double countsPerRev;
        private final double drumDiameter;

        public GenericCountsImpl(Supplier<Double> getCounts, DoubleConsumer setCounts,
                                 double gearRatio, double countsPerRev, double drumDiameter) {
            this.countsSupplier = getCounts;
            this.countsSetter = setCounts;
            this.gearRatio = gearRatio;
            this.countsPerRev = countsPerRev;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public boolean isAbsolute() { return false; } // Generic assumed relative

        @Override
        public double getPositionMeters() {
            double counts = countsSupplier.get();
            double deg = EncoderConversion.encoderUnitsToDegrees(counts, gearRatio, countsPerRev);
            return EncoderConversion.degreesToMeters(deg, drumDiameter);
        }

        @Override
        public void setPositionMeters(double meters) {
            double circumference = Math.PI * drumDiameter;
            double rotations = meters / circumference;
            double deg = rotations * 360.0;
            double counts = deg * countsPerRev * gearRatio / 360.0;
            countsSetter.accept(counts);
        }
    }

    /** Generic mechanism-rotations implementation. */
    class GenericMechanismRotationsImpl implements EncoderWrapper {
        private final Supplier<Double> rotationsSupplier;
        private final DoubleConsumer rotationsSetter;
        private final double drumDiameter;

        public GenericMechanismRotationsImpl(Supplier<Double> rotationsSupplier,
                                             DoubleConsumer rotationsSetter,
                                             double drumDiameter) {
            this.rotationsSupplier = rotationsSupplier;
            this.rotationsSetter = rotationsSetter;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public double getPositionMeters() {
            double rotations = rotationsSupplier.get();
            return rotations * Math.PI * drumDiameter;
        }

        @Override
        public void setPositionMeters(double meters) {
            double circumference = Math.PI * drumDiameter;
            double rotations = meters / circumference;
            rotationsSetter.accept(rotations);
        }
    }

    /** Generic absolute rotations-based implementation (sensor rotations in [0,1)). */
    class GenericAbsoluteRotationsImpl implements EncoderWrapper {
        private final Supplier<Double> absoluteRotationsSupplier;
        private final double gearRatio;
        private final double drumDiameter;
        private double offsetMeters = 0.0;

        public GenericAbsoluteRotationsImpl(Supplier<Double> absoluteRotationsSupplier,
                                            double gearRatio,
                                            double drumDiameter) {
            this.absoluteRotationsSupplier = absoluteRotationsSupplier;
            this.gearRatio = gearRatio;
            this.drumDiameter = drumDiameter;
        }

        @Override
        public boolean isAbsolute() { return true; }

        @Override
        public double getPositionMeters() {
            double sensorRot = absoluteRotationsSupplier.get(); // 0..1 sensor rotations
            double mechRot = sensorRot / gearRatio;
            double meters = mechRot * Math.PI * drumDiameter;
            return meters + offsetMeters;
        }

        @Override
        public void setPositionMeters(double meters) {
            // Cannot set absolute sensor; emulate by offset
            offsetMeters = meters - getPositionMeters();
        }
    }
}
