package ca.team4308.coprocessor;

import ca.team4308.absolutelib.network.task.server.TaskHandler;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class TrajectoryTaskHandler implements TaskHandler {
    private final TrajectoryWrapper wrapper;
    private static final ObjectMapper mapper = new ObjectMapper();

    public TrajectoryTaskHandler(TrajectoryWrapper wrapper) {
        this.wrapper = wrapper;
    }

    @Override
    public JsonNode handle(JsonNode input) throws Exception {
        TrajectoryRequest req = mapper.treeToValue(input, TrajectoryRequest.class);
        TrajectoryResponse res = wrapper.solve(req);
        return mapper.valueToTree(res);
    }
}
