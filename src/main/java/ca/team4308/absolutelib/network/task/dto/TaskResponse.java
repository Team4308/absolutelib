package ca.team4308.absolutelib.network.task.dto;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.JsonNode;

public class TaskResponse {
    @JsonProperty("request_id")
    public String requestId;

    @JsonProperty("task_type")
    public String taskType;

    @JsonProperty("timestamp")
    public double timestamp;

    @JsonProperty("success")
    public boolean success;

    @JsonProperty("error_message")
    public String errorMessage;

    @JsonProperty("payload")
    public JsonNode payload;

    public TaskResponse() {}

    public static TaskResponse success(String requestId, String taskType, double timestamp, JsonNode payload) {
        TaskResponse res = new TaskResponse();
        res.requestId = requestId;
        res.taskType = taskType;
        res.timestamp = timestamp;
        res.success = true;
        res.payload = payload;
        return res;
    }

    public static TaskResponse error(String requestId, String taskType, double timestamp, String errorMessage) {
        TaskResponse res = new TaskResponse();
        res.requestId = requestId;
        res.taskType = taskType;
        res.timestamp = timestamp;
        res.success = false;
        res.errorMessage = errorMessage;
        return res;
    }
}
