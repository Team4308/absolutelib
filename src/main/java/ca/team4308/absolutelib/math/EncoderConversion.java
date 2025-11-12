package ca.team4308.absolutelib.math;

public class EncoderConversion {
    
    /*
     * Convert encoder units to degrees
     * @param encoderUnits - The encoder units to convert
     * @param gearRatio - The gear ratio of the mechanism
     * @param pulsesPerRevolution - The number of pulses per revolution of the encoder
     * @return The converted degrees
     */
    public static double encoderUnitsToDegrees(double encoderUnits, double gearRatio, double pulsesPerRevolution) {
        return (encoderUnits / pulsesPerRevolution) * (360.0 / gearRatio);
    }

    /*
     * Convert degrees to meters 
     * @param degrees - The degrees to convert
     * @param drumDiameter - The diameter of the drum
     * @return The converted meters
     */
    public static double degreesToMeters(double degrees, double drumDiameter) {
        return (degrees / 360.0) * (Math.PI * drumDiameter);
    }


    /*
     * Convert meters to degrees
     * @param meters - The meters to convert
     * @param drumDiameter - The diameter of the drum
     * @return The converted degrees
     */
    public static double metersToDegrees(double meters, double drumDiameter) {
        return (meters / (Math.PI * drumDiameter)) * 360.0;
    }




}
