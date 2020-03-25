package ca.team4308.absolutelib.path.tankfollower;

public class TankFollowerSettings {
    public final double kEncoderCountsPerRotation;
    public final double kGearRatio;

    public final TankFollowerGains leftGains;
    public final TankFollowerGains rightGains;
    public final TankFollowerGains turnGains;

    public final double period;

    public TankFollowerSettings(double _kEncoderCountsPerRotation, double _kGearRatio, TankFollowerGains _leftGains,
    TankFollowerGains _rightGains, TankFollowerGains _turnGains, double _period) {
        kEncoderCountsPerRotation = _kEncoderCountsPerRotation;
        kGearRatio = _kGearRatio;

        leftGains = _leftGains;
        rightGains = _rightGains;
        turnGains = _turnGains;

        period = _period;
    }
}