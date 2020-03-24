package ca.team4308.absolutelib.path;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.drive.TankDriveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * UltraPathFollower
 * Paths must be generated with p,v,a,h
 */
public class UltraPathFollower extends CommandBase {
    private final double[][] leftPath, rightPath;

    private final PathFollowSettings settings;
    private final TankDriveSubsystem m_subsystem;

    PIDController turnController;

    private volatile boolean isFinished = false;
    private volatile int currentPoint = 0;

    private volatile double leftPrevError = 0.0;
    private volatile double rightPrevError = 0.0;

    /**
     * UltraPathFollower, a pd based motion profile follower for tank drive drivetrains
     * Paths must be generated with p,v,a,h
     * 
     * Runs as a command
     * 
     * @param profileFilename Motion profile name
     * @param settings Path follower settings
     * @param subsystem Tank Drive Subsystem
     */
    public UltraPathFollower(String profileFilename, PathFollowSettings settings, TankDriveSubsystem subsystem) {
        leftPath = loadPath(profileFilename + "_left");
        rightPath = loadPath(profileFilename + "_right");

        this.settings = settings;
        this.m_subsystem = subsystem;

        turnController = new PIDController(settings.turnGains.kP, settings.turnGains.kI, settings.turnGains.kD);
        turnController.enableContinuousInput(-180, 180);
        turnController.setTolerance(settings.turnGains.tolerance / 5.4);
        turnController.setSetpoint(subsystem.getAhrs().getYaw());

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.resetSensors();
        m_subsystem.selectProfileSlot(settings.profileSlot);

        isFinished = false;
        currentPoint = 0;

        if (leftPath.length != rightPath.length) {
            DriverStation.reportError("Left Path Length Is Not The Same As Right Path Length", false);
            this.cancel();
            return;
        }

        new Thread(() -> {
            double lastTime = 0.0;

            while (!isFinished && DriverStation.getInstance().isEnabled()) {
                if (Timer.getFPGATimestamp() >= lastTime + settings.period) {
                    lastTime = Timer.getFPGATimestamp();
                    calculate();
                }
                try {
                    Thread.sleep(2);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }

    protected synchronized void calculate() {
        if (currentPoint < leftPath.length) {
            double leftGoalPos = leftPath[currentPoint][0];
            double leftGoalVel = leftPath[currentPoint][1];
            double leftGoalAcc = leftPath[currentPoint][2];
            double leftGoalAbsHead = leftPath[currentPoint][3];
            double leftGoalRelHead = leftPath[currentPoint][4];

            double rightGoalPos = rightPath[currentPoint][0];
            double rightGoalVel = rightPath[currentPoint][1];
            double rightGoalAcc = rightPath[currentPoint][2];
            double rightGoalAbsHead = rightPath[currentPoint][3];
            double rightGoalRelHead = leftPath[currentPoint][4];

            double avgAbsHeading = (leftGoalAbsHead + rightGoalAbsHead) / 2;
            double avgRelHeading = (leftGoalRelHead + rightGoalRelHead) / 2;

            //double turnEncoderAmt = (((22.68 * Math.PI / settings.kEncoderCountsPerRotation) / settings.kGearRatio) * (avgRelHeading / 360.0));

            //double adjLeftGoalPos = leftGoalPos - turnEncoderAmt;
            //double adjRightGoalPos = rightGoalPos + turnEncoderAmt;

            double leftError = ((leftGoalPos * 12) / settings.kGearRatio) - m_subsystem.getLeftSensorPosition();
            double leftDerivError = ((leftError - leftPrevError) / settings.period) - leftGoalVel;

            double rightError = ((rightGoalPos * 12) / settings.kGearRatio) - m_subsystem.getRightSensorPosition();
            double rightDerivError = ((rightError - rightPrevError) / settings.period) - rightGoalVel;

            double leftOutput = (settings.leftGains.kP * leftError) + (settings.leftGains.kD * leftDerivError) + (settings.leftGains.kV * leftGoalVel) + (settings.leftGains.ka * leftGoalAcc);
            double rightOutput = (settings.rightGains.kP * rightError) + (settings.rightGains.kD * rightDerivError) + (settings.rightGains.kV * rightGoalVel) + (settings.rightGains.ka * rightGoalAcc);

            //double turnOutput = bbbDoubleUtils.clamp(turnController.calculate(m_subsystem.getAhrs().getAngle(), avgHeading), -1, 1);

            //leftOutput += turnOutput;
            //rightOutput -= turnOutput;

            leftOutput = DoubleUtils.clamp(leftOutput, -1.0, 1.0);
            rightOutput = DoubleUtils.clamp(rightOutput, -1.0, 1.0);

            leftPrevError = leftError;
            rightPrevError = rightError;

            m_subsystem.setMotorOutputPercent(leftOutput, rightOutput);

            currentPoint++;
        } else {
            isFinished = true;
        }
    }

    double[][] loadPath(String profileFilename) {
        File file = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/" + profileFilename + ".csv");

        try (BufferedReader csvReader = new BufferedReader(new FileReader(file))) {
            ArrayList<double[]> pointArray = new ArrayList<double[]>();

            String line;
            while ((line = csvReader.readLine()) != null) {
                double[] point = Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
                pointArray.add(point);
            }

            double[][] doubleArray = new double[pointArray.size()][5];
            pointArray.toArray(doubleArray);

            return doubleArray;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }
}