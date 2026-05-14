package ca.team4308.coprocessor;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;
import ca.team4308.absolutelib.math.trajectories.TrajectoryResult;

public class NT4Publisher {

    private final NetworkTableInstance inst;
    private final NetworkTable table;
    
    private final BooleanPublisher connectedPub;
    private final DoublePublisher solverTimePub;
    private final IntegerPublisher totalRequestsPub;
    private final IntegerPublisher droppedPacketsPub;

    private final DoublePublisher inRobotXPub;
    private final DoublePublisher inRobotYPub;
    private final DoublePublisher resPitch;
    private final DoublePublisher resYaw;
    private final DoublePublisher resRpm;
    private final BooleanPublisher resValid;
    private final DoublePublisher tcpLatencyMs;
    
    private final IntegerPublisher activeConfigVersionIdPub;
    private final DoublePublisher filteredLatencyMsPub;

    private final IntegerPublisher lossyCountPub;
    private final DoubleArrayPublisher lossyPathXPub;
    private final DoubleArrayPublisher lossyPathYPub;
    private final DoubleArrayPublisher lossyPathZPub;
    private final StringPublisher lossySvgPathPub;

    // Flywheel
    private final DoublePublisher fwWheelRpmPub;
    private final DoublePublisher fwMotorRpmPub;
    private final DoublePublisher fwPowerPub;
    private final DoublePublisher fwSpinUpPub;
    private final DoublePublisher fwCurrentPub;
    private final DoublePublisher fwStoredEnergyPub;
    private final DoublePublisher fwContactMsPub;
    private final DoublePublisher fwBallSpinPub;
    private final DoublePublisher fwSlipRatioPub;
    private final DoublePublisher fwEfficiencyPub;
    private final BooleanPublisher fwAchievablePub;

    // Solver
    private final StringPublisher solverModePub;
    private final IntegerPublisher solverIterPub;
    private final IntegerPublisher solverTestedPub;
    private final IntegerPublisher solverAcceptedPub;

    // Metrics
    private final DoublePublisher confidencePub;
    private final DoublePublisher marginErrorPub;
    private final DoublePublisher maxHeightPub;
    private final DoublePublisher tofPub;
    private final DoublePublisher distancePub;

    // Discrete
    private final BooleanPublisher discreteValidPub;
    private final DoublePublisher discreteRpmPub;
    private final DoublePublisher discretePitchPub;
    private final DoublePublisher discreteScorePub;
    
    public NT4Publisher() {
        inst = NetworkTableInstance.getDefault();
        inst.startClient4("trajectory-coprocessor");
        
        String addr = Config.getRobotAddress();
        System.out.println("NT4 Connecting to " + addr);
        inst.setServer(addr);
        
        table = inst.getTable("TrajectoryCoprocessor");
        connectedPub = table.getBooleanTopic("Status/Connected").publish();
        solverTimePub = table.getDoubleTopic("Status/SolverTimeMs").publish();
        totalRequestsPub = table.getIntegerTopic("Metrics/TotalRequests").publish();
        droppedPacketsPub = table.getIntegerTopic("Metrics/DroppedPackets").publish();
        inRobotXPub = table.getDoubleTopic("Input/RobotX").publish();
        inRobotYPub = table.getDoubleTopic("Input/RobotY").publish();
        resPitch = table.getDoubleTopic("Output/PitchDeg").publish();
        resYaw = table.getDoubleTopic("Output/YawDeg").publish();
        resRpm = table.getDoubleTopic("Output/RPM").publish();
        resValid = table.getBooleanTopic("Output/Valid").publish();
        tcpLatencyMs = table.getDoubleTopic("Status/TCPLatencyMs").publish();
        
        activeConfigVersionIdPub = table.getIntegerTopic("Status/ActiveConfigVersionId").publish();
        filteredLatencyMsPub = table.getDoubleTopic("Status/FilteredLatencyMs").publish();

        lossyCountPub = table.getIntegerTopic("Lossy/Count").publish();
        lossyPathXPub = table.getDoubleArrayTopic("Lossy/PathX").publish();
        lossyPathYPub = table.getDoubleArrayTopic("Lossy/PathY").publish();
        lossyPathZPub = table.getDoubleArrayTopic("Lossy/PathZ").publish();
        lossySvgPathPub = table.getStringTopic("Lossy/SvgPath").publish();

        // Flywheel
        fwWheelRpmPub = table.getDoubleTopic("Flywheel/WheelRPM").publish();
        fwMotorRpmPub = table.getDoubleTopic("Flywheel/MotorRPM").publish();
        fwPowerPub = table.getDoubleTopic("Flywheel/Power").publish();
        fwSpinUpPub = table.getDoubleTopic("Flywheel/SpinUpTime").publish();
        fwCurrentPub = table.getDoubleTopic("Flywheel/Current").publish();
        fwStoredEnergyPub = table.getDoubleTopic("Flywheel/StoredEnergy").publish();
        fwContactMsPub = table.getDoubleTopic("Flywheel/ContactTimeMs").publish();
        fwBallSpinPub = table.getDoubleTopic("Flywheel/BallSpin").publish();
        fwSlipRatioPub = table.getDoubleTopic("Flywheel/SlipRatio").publish();
        fwEfficiencyPub = table.getDoubleTopic("Flywheel/Efficiency").publish();
        fwAchievablePub = table.getBooleanTopic("Flywheel/Achievable").publish();

        // Solver
        solverModePub = table.getStringTopic("Solver/Mode").publish();
        solverIterPub = table.getIntegerTopic("Solver/Iterations").publish();
        solverTestedPub = table.getIntegerTopic("Solver/TotalTested").publish();
        solverAcceptedPub = table.getIntegerTopic("Solver/Accepted").publish();

        // Metrics
        confidencePub = table.getDoubleTopic("Metrics/Confidence").publish();
        marginErrorPub = table.getDoubleTopic("Metrics/MarginOfError").publish();
        maxHeightPub = table.getDoubleTopic("Metrics/MaxHeight").publish();
        tofPub = table.getDoubleTopic("Metrics/TOF").publish();
        distancePub = table.getDoubleTopic("Metrics/Distance").publish();

        // Discrete
        discreteValidPub = table.getBooleanTopic("Discrete/Valid").publish();
        discreteRpmPub = table.getDoubleTopic("Discrete/RPM").publish();
        discretePitchPub = table.getDoubleTopic("Discrete/PitchDeg").publish();
        discreteScorePub = table.getDoubleTopic("Discrete/Score").publish();
    }

    public void update(TrajectoryRequest req, TrajectoryResponse res, long latency, TrajectoryResult trajResult, boolean isConnected, double solverTime, long totalRequests, long droppedPackets, TCPServer tcpServer) {
        connectedPub.set(isConnected);
        solverTimePub.set(solverTime);
        totalRequestsPub.set(totalRequests);
        droppedPacketsPub.set(droppedPackets);

        if (req != null) {
            inRobotXPub.set(req.robotX);
            inRobotYPub.set(req.robotY);
        }
        
        if (res != null) {
            resPitch.set(res.pitchDegrees);
            resYaw.set(res.yawDegrees);
            resRpm.set(res.rpm);
            resValid.set(res.valid);
        }
        
        tcpLatencyMs.set(latency);
        
        activeConfigVersionIdPub.set(tcpServer.getActiveConfigVersionId());
        filteredLatencyMsPub.set(tcpServer.getFilteredLatencyMs());

        ca.team4308.absolutelib.math.trajectories.network.FullTelemetryPacket fullPacket = tcpServer.latestFullTelemetry.get();
        if (fullPacket != null) {
            // Lossy Path
            if (fullPacket.pathCount > 0) {
                lossyCountPub.set(fullPacket.pathCount);
                lossyPathXPub.set(fullPacket.flightPathX);
                lossyPathYPub.set(fullPacket.flightPathY);
                lossyPathZPub.set(fullPacket.flightPathZ);
                
                StringBuilder svgPath = new StringBuilder();
                for (int i = 0; i < fullPacket.pathCount; i++) {
                    if (i == 0) {
                        svgPath.append("M ").append(fullPacket.flightPathX[i]).append(' ').append(fullPacket.flightPathY[i]);
                    } else {
                        svgPath.append(" L ").append(fullPacket.flightPathX[i]).append(' ').append(fullPacket.flightPathY[i]);
                    }
                }
                lossySvgPathPub.set(svgPath.toString());
            } else {
                lossyCountPub.set(0);
                lossyPathXPub.set(new double[0]);
                lossyPathYPub.set(new double[0]);
                lossyPathZPub.set(new double[0]);
                lossySvgPathPub.set("");
            }

            // Flywheel
            fwWheelRpmPub.set(fullPacket.fwWheelRpm);
            fwMotorRpmPub.set(fullPacket.fwMotorRpm);
            fwPowerPub.set(fullPacket.fwMotorPower);
            fwSpinUpPub.set(fullPacket.fwSpinUpSec);
            fwCurrentPub.set(fullPacket.fwCurrentAmps);
            fwStoredEnergyPub.set(fullPacket.fwStoredJoules);
            fwContactMsPub.set(fullPacket.fwContactMs);
            fwBallSpinPub.set(fullPacket.fwBallSpinRpm);
            fwSlipRatioPub.set(fullPacket.fwSlipRatio);
            fwEfficiencyPub.set(fullPacket.fwEfficiency);
            fwAchievablePub.set(fullPacket.fwAchievable);

            // Solver
            solverModePub.set(fullPacket.trMode);
            solverIterPub.set(fullPacket.trIterations);
            solverTestedPub.set(fullPacket.trTotalTested);
            solverAcceptedPub.set(fullPacket.trAccepted);

            // Metrics
            confidencePub.set(fullPacket.confidence);
            marginErrorPub.set(fullPacket.metMarginError);
            maxHeightPub.set(fullPacket.metMaxHeight);
            tofPub.set(fullPacket.metTof);
            distancePub.set(fullPacket.metDistance);

            // Discrete
            discreteValidPub.set(fullPacket.dsValid);
            discreteRpmPub.set(fullPacket.dsRpm);
            discretePitchPub.set(fullPacket.dsPitchDeg);
            discreteScorePub.set(fullPacket.dsScore);
        } else {
            lossyCountPub.set(0);
            lossyPathXPub.set(new double[0]);
            lossyPathYPub.set(new double[0]);
            lossyPathZPub.set(new double[0]);
            lossySvgPathPub.set("");
        }
    }
}
