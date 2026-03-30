package ca.team4308.absolutelib.network.task.server;

import java.util.concurrent.ConcurrentHashMap;

public class TaskRegistry {
    private final ConcurrentHashMap<String, TaskHandler> handlers = new ConcurrentHashMap<>();
    
    public void register(String taskType, TaskHandler handler) {
        handlers.put(taskType, handler);
    }
    
    public TaskHandler getHandler(String taskType) {
        return handlers.get(taskType);
    }
}
