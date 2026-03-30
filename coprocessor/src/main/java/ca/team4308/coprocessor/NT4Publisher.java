package ca.team4308.coprocessor;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.IntegerPublisher;
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
    }

    public void update(TrajectoryRequest req, TrajectoryResponse res, long latency, TrajectoryResult trajResult, boolean isConnected, double solverTime, long totalRequests, long droppedPackets) {
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
    }
}
