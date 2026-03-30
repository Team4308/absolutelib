package ca.team4308.absolutelib.network.task.client;

import ca.team4308.absolutelib.network.task.dto.TaskResponse;
import com.fasterxml.jackson.databind.ObjectMapper;

public class TaskHandle<T> {
    private final String requestId;
    private final Class<T> expectedResponseType;
    private final double creationTimeSec;
    private static final ObjectMapper mapper = new ObjectMapper();

    private volatile boolean done = false;
    private volatile boolean success = false;
    private volatile T result = null;
    private volatile String errorMessage = null;

    public TaskHandle(String requestId, Class<T> expectedResponseType, double creationTimeSec) {
        this.requestId = requestId;
        this.expectedResponseType = expectedResponseType;
        this.creationTimeSec = creationTimeSec;
    }

    public String getRequestId() {
        return requestId;
    }

    public boolean isDone() {
        return done;
    }

    public boolean isSuccess() {
        return success;
    }

    public T get() {
        return result;
    }

    public String getErrorMessage() {
        return errorMessage;
    }

    public boolean isStale(double currentTimeSec, double timeoutSec) {
        return (currentTimeSec - creationTimeSec) > timeoutSec;
    }

    protected void complete(TaskResponse response) {
        if (response.success && response.payload != null) {
            try {
                this.result = mapper.treeToValue(response.payload, expectedResponseType);
                this.success = true;
            } catch (Exception e) {
                this.success = false;
                this.errorMessage = "Failed to map payload to " + expectedResponseType.getSimpleName() + ": " + e.getMessage();
            }
        } else {
            this.success = false;
            this.errorMessage = response.errorMessage;
        }
        this.done = true;
    }

    protected void completeExceptionally(String error) {
        this.success = false;
        this.errorMessage = error;
        this.done = true;
    }
}
