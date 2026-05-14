package ca.team4308.coprocessor;

import ca.team4308.absolutelib.network.task.server.TaskHandler;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class TrajectoryTaskHandler implements TaskHandler {
    private final TrajectoryWrapper wrapper;
    private final TCPServer tcpServer;
    private static final ObjectMapper mapper = new ObjectMapper();
    private long lastLogTime = 0;
    private double lastLogX = 0;
    private double lastLogY = 0;

    public TrajectoryTaskHandler(TrajectoryWrapper wrapper, TCPServer tcpServer) {
        this.wrapper = wrapper;
        this.tcpServer = tcpServer;
    }

    @Override
    public JsonNode handle(JsonNode input) throws Exception {
        long start = System.currentTimeMillis();
        tcpServer.isConnected.set(true);
        tcpServer.totalRequests.incrementAndGet();

        TrajectoryRequest req = mapper.treeToValue(input, TrajectoryRequest.class);
        if (req.battery > 0) {
            tcpServer.setBattery(req.battery);
        }
        tcpServer.latestRequest.set(req);

        TrajectoryResponse res = wrapper.solve(req);
        tcpServer.updateTelemetry(req, res, wrapper.getShooterSystem().getLastTrajectoryResult(), System.currentTimeMillis() - start);

        if (Config.LOG_TO_FILE) {
            long now = System.currentTimeMillis();
            // Throttle logging to max 2 times a second, AND only if the robot actually moved
            // TODO: Send update on move command, check timedRobotCommand for more direct logging instead of position change + time 
            if (now - lastLogTime >= 500) { 
                boolean moved = Math.abs(req.robotX - lastLogX) > 0.05 || Math.abs(req.robotY - lastLogY) > 0.05;
                if (moved) {
                    ReplayLogger.log(req, res);
                    lastLogTime = now;
                    lastLogX = req.robotX;
                    lastLogY = req.robotY;
                }
            }
        }

        return mapper.valueToTree(res);
    }
}
