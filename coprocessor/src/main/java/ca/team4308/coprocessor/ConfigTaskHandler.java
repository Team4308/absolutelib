package ca.team4308.coprocessor;

import ca.team4308.absolutelib.network.task.server.TaskHandler;
import ca.team4308.absolutelib.math.trajectories.network.ConfigurationPacket;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.HashMap;
import java.util.Map;

public class ConfigTaskHandler implements TaskHandler {
    private final TrajectoryWrapper wrapper;
    private final TCPServer tcpServer;
    private static final ObjectMapper mapper = new ObjectMapper();

    public ConfigTaskHandler(TrajectoryWrapper wrapper, TCPServer tcpServer) {
        this.wrapper = wrapper;
        this.tcpServer = tcpServer;
    }

    @Override
    public JsonNode handle(JsonNode input) throws Exception {
        System.out.println("[ConfigTask] Received CONFIG_UPDATE request");
        
        ConfigurationPacket config = mapper.treeToValue(input, ConfigurationPacket.class);
        wrapper.updateConfiguration(config);
        
        tcpServer.latestConfig.set(config);
        tcpServer.activeConfigVersionId.set(config.configVersionId);
        
        Map<String, Object> ack = new HashMap<>();
        ack.put("status", "success");
        ack.put("version", config.configVersionId);
        ack.put("timestamp", System.currentTimeMillis() / 1000.0);
        
        return mapper.valueToTree(ack);
    }
}
