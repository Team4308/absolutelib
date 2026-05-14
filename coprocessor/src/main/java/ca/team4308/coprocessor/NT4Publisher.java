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

        ca.team4308.absolutelib.math.trajectories.network.LossyDataPacket lossyPacket = tcpServer.latestLossyPacket.get();
        if (lossyPacket != null && lossyPacket.flightPath != null && !lossyPacket.flightPath.isEmpty()) {
            int count = lossyPacket.flightPath.size();
            double[] xs = new double[count];
            double[] ys = new double[count];
            double[] zs = new double[count];
            StringBuilder svgPath = new StringBuilder();
            for (int i = 0; i < count; i++) {
                ca.team4308.absolutelib.math.trajectories.impl.Pose3d pose = lossyPacket.flightPath.get(i);
                xs[i] = pose.getTranslation().x;
                ys[i] = pose.getTranslation().y;
                zs[i] = pose.getTranslation().z;
                if (i == 0) {
                    svgPath.append("M ").append(xs[i]).append(' ').append(ys[i]);
                } else {
                    svgPath.append(" L ").append(xs[i]).append(' ').append(ys[i]);
                }
            }
            lossyCountPub.set(count);
            lossyPathXPub.set(xs);
            lossyPathYPub.set(ys);
            lossyPathZPub.set(zs);
            lossySvgPathPub.set(svgPath.toString());
        } else {
            lossyCountPub.set(0);
            lossyPathXPub.set(new double[0]);
            lossyPathYPub.set(new double[0]);
            lossyPathZPub.set(new double[0]);
            lossySvgPathPub.set("");
        }
    }
}
