package ca.team4308.absolutelib.network.task.client;

import ca.team4308.absolutelib.network.task.dto.TaskRequest;
import ca.team4308.absolutelib.network.task.dto.TaskResponse;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.atomic.AtomicBoolean;

public class CoprocessorClient implements Runnable {
    private final String host;
    private final int port;
    private final int telemetryPort;
    private static final ObjectMapper mapper = new ObjectMapper();
    
    // Thread-safe map holding handles to all currently submitted but unfinished remote tasks.
    private final ConcurrentHashMap<String, TaskHandle<?>> pendingTasks = new ConcurrentHashMap<>();
    
    // The active socket writer thread output
    private final AtomicReference<PrintWriter> socketOut = new AtomicReference<>(null);
    private final AtomicBoolean connected = new AtomicBoolean(false);

    private java.util.function.Consumer<JsonNode> telemetryListener;

    public CoprocessorClient(String host, int port) {
        this(host, port, 5806);
    }

    public CoprocessorClient(String host, int port, int telemetryPort) {
        this.host = host;
        this.port = port;
        this.telemetryPort = telemetryPort;
    }

    public void setTelemetryListener(java.util.function.Consumer<JsonNode> listener) {
        this.telemetryListener = listener;
    }

    /**
     * Submits a payload to the coprocessor server asynchronously without blocking.
     */
    public <T> TaskHandle<T> submitTask(String taskType, Object payloadObj, Class<T> responseClass, double currentTimeSec) {
        String reqId = UUID.randomUUID().toString();
        TaskHandle<T> handle = new TaskHandle<>(reqId, responseClass, currentTimeSec);
        
        try {
            JsonNode payloadNode = mapper.valueToTree(payloadObj);
            TaskRequest req = new TaskRequest(reqId, taskType, currentTimeSec, payloadNode);
            pendingTasks.put(reqId, handle);
            
            PrintWriter out = socketOut.get();
            if (out != null) {
                String json = mapper.writeValueAsString(req);
                synchronized (out) {
                    out.println(json);
                }
            } else {
                handle.completeExceptionally("Client disconnected");
                pendingTasks.remove(reqId);
            }
        } catch (Exception e) {
            handle.completeExceptionally("Serialization error: " + e.getMessage());
            pendingTasks.remove(reqId);
        }
        
        return handle;
    }

    /**
     * Call this periodically in the robot loop to drop tasks that never received a response 
     * causing Memory Leaks on the roboRIO.
     */
    public void pruneStaleTasks(double currentTimeSec, double timeoutSec) {
        pendingTasks.entrySet().removeIf(entry -> {
            TaskHandle<?> handle = entry.getValue();
            if (handle.isStale(currentTimeSec, timeoutSec)) {
                handle.completeExceptionally("Timeout");
                return true;
            }
            return false;
        });
    }

    @Override
    public void run() {
        // Start UDP telemetry listener thread
        Thread telemetryThread = new Thread(this::runTelemetryLoop);
        telemetryThread.setDaemon(true);
        telemetryThread.setName("Coprocessor-UDP-Telemetry");
        telemetryThread.start();

        while (!Thread.currentThread().isInterrupted()) {
            try (Socket socket = new Socket(host, port);
                 PrintWriter out = new PrintWriter(socket.getOutputStream(), true);
                 BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()))) {
                 
                socket.setTcpNoDelay(true);
                socketOut.set(out);
                connected.set(true);
                System.out.println("Generic CoprocessorClient Connected to " + host + ":" + port);

                if (pendingTasks.isEmpty()) {
                    System.out.println("Generic CoprocessorClient: no pending tasks yet");
                }

                String line;
                System.out.println("Generic CoprocessorClient waiting for lines...");
                while ((line = in.readLine()) != null) {
                    try {
                        TaskResponse res = mapper.readValue(line, TaskResponse.class);
                        TaskHandle<?> handle = pendingTasks.remove(res.requestId);
                        if (handle != null) {
                            handle.complete(res);
                        }
                    } catch (Exception e) {
                        System.err.println("Failed parsing response: " + e.getMessage());
                    }
                }
            } catch (Exception e) {
                // Connection physically dropped natively or remote terminated. Will automatically reconnect.
                System.err.println("Generic CoprocessorClient Connection Error: " + e.getMessage());
            } finally {
                socketOut.set(null);
                connected.set(false);
                
                // Forcibly clear any tasks that were waiting on responses over this socket. 
                // We know they won't seamlessly flow onto the new reconn socket as state represents discrete packets.
                for (TaskHandle<?> handle : pendingTasks.values()) {
                    handle.completeExceptionally("Socket dropped");
                }
                pendingTasks.clear();
                
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    private void runTelemetryLoop() {
        byte[] buffer = new byte[65535];
        try (java.net.DatagramSocket socket = new java.net.DatagramSocket(telemetryPort)) {
            System.out.println("UDP Telemetry Listener started on port " + telemetryPort);
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    java.net.DatagramPacket packet = new java.net.DatagramPacket(buffer, buffer.length);
                    socket.receive(packet);
                    
                    if (telemetryListener != null) {
                        JsonNode node = mapper.readTree(packet.getData(), 0, packet.getLength());
                        telemetryListener.accept(node);
                    }
                } catch (Exception e) {
                    // System.err.println("Error receiving UDP telemetry: " + e.getMessage());
                }
            }
        } catch (Exception e) {
            System.err.println("UDP Telemetry Listener crashed: " + e.getMessage());
        }
    }

    public boolean isConnected() {
        return connected.get();
    }
}
