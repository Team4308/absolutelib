package ca.team4308.absolutelib.path;

public class PathFollowSettings {
    public final double kEncoderCountsPerRotation;
    public final double kGearRatio;

    public final PathGains leftGains;
    public final PathGains rightGains;
    public final PathGains turnGains;

    public final int profileSlot;
    public final double period;

    public PathFollowSettings(double _kEncoderCountsPerRotation, double _kGearRatio, PathGains _leftGains,
            PathGains _rightGains, PathGains _turnGains, int _profileSlot, double _period) {
        kEncoderCountsPerRotation = _kEncoderCountsPerRotation;
        kGearRatio = _kGearRatio;

        leftGains = _leftGains;
        rightGains = _rightGains;
        turnGains = _turnGains;

        profileSlot = _profileSlot;
        period = _period;
    }
}