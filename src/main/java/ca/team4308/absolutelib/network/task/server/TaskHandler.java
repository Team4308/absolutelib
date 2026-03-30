package ca.team4308.absolutelib.network.task.server;

import com.fasterxml.jackson.databind.JsonNode;

public interface TaskHandler {
    JsonNode handle(JsonNode input) throws Exception;
}
