package ca.team4308.absolutelib.wrapper.drive;

import com.kauailabs.navx.frc.AHRS;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;

public abstract class TankDriveSubsystem extends MotoredSubsystem {
    public abstract void setMotorOutputPercent(double left, double right);

    public abstract void resetSensors();

    public abstract double getLeftSensorPosition();
    public abstract double getRightSensorPosition();
    public abstract double getLeftSensorVelocity();
    public abstract double getRightSensorVelocity();

    public abstract AHRS getAhrs();
}