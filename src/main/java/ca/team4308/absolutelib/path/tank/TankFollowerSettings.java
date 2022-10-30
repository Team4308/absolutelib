package ca.team4308.absolutelib.path.tank;

import ca.team4308.absolutelib.path.Gains;

public class TankFollowerSettings {
    public final double kEncoderCountsPerRotation;
    public final double kGearRatio;

    public final Gains leftGains;
    public final Gains rightGains;
    public final Gains turnGains;

    public final int profileSlot;
    public final double period;

    public TankFollowerSettings(double _kEncoderCountsPerRotation, double _kGearRatio, Gains _leftGains,
            Gains _rightGains, Gains _turnGains, int _profileSlot, double _period) {
        kEncoderCountsPerRotation = _kEncoderCountsPerRotation;
        kGearRatio = _kGearRatio;

        leftGains = _leftGains;
        rightGains = _rightGains;
        turnGains = _turnGains;

        profileSlot = _profileSlot;
        period = _period;
    }
}
