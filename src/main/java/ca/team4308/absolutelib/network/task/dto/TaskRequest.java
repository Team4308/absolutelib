package ca.team4308.absolutelib.network.task.dto;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.JsonNode;

public class TaskRequest {
    @JsonProperty("request_id")
    public String requestId;

    @JsonProperty("task_type")
    public String taskType;

    @JsonProperty("timestamp")
    public double timestamp;

    @JsonProperty("payload")
    public JsonNode payload;

    public TaskRequest() {}

    public TaskRequest(String requestId, String taskType, double timestamp, JsonNode payload) {
        this.requestId = requestId;
        this.taskType = taskType;
        this.timestamp = timestamp;
        this.payload = payload;
    }
}
