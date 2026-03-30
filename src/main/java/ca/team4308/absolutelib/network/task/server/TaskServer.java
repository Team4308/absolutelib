package ca.team4308.absolutelib.network.task.server;

import ca.team4308.absolutelib.network.task.dto.TaskRequest;
import ca.team4308.absolutelib.network.task.dto.TaskResponse;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class TaskServer implements Runnable {
    private final int port;
    private final TaskRegistry registry;
    private final ExecutorService threadPool;
    private static final ObjectMapper mapper = new ObjectMapper();

    public TaskServer(int port, TaskRegistry registry, int workerCount) {
        this.port = port;
        this.registry = registry;
        this.threadPool = Executors.newFixedThreadPool(workerCount);
    }

    @Override
    public void run() {
        try (ServerSocket serverSocket = new ServerSocket(port)) {
            System.out.println("TaskServer listening on port " + port);
            while (!Thread.currentThread().isInterrupted()) {
                Socket clientSocket = serverSocket.accept();
                clientSocket.setTcpNoDelay(true);
                // Handle each client connection in an isolated thread
                Thread clientThread = new Thread(new ClientHandler(clientSocket));
                clientThread.setDaemon(true);
                clientThread.start();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private class ClientHandler implements Runnable {
        private final Socket socket;

        public ClientHandler(Socket socket) {
            this.socket = socket;
        }

        @Override
        public void run() {
            try (BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                 PrintWriter out = new PrintWriter(socket.getOutputStream(), true)) {

                String line;
                while ((line = in.readLine()) != null) {
                    final String inputLine = line;
                    // Hand off logic parsing to Worker Pool
                    threadPool.submit(() -> {
                        TaskResponse response = null;
                        TaskRequest request = null;
                        try {
                            request = mapper.readValue(inputLine, TaskRequest.class);
                            TaskHandler handler = registry.getHandler(request.taskType);

                            if (handler == null) {
                                response = TaskResponse.error(request.requestId, request.taskType, request.timestamp, "Unknown task type");
                            } else {
                                JsonNode resultNode = handler.handle(request.payload);
                                response = TaskResponse.success(request.requestId, request.taskType, request.timestamp, resultNode);
                            }
                        } catch (Exception e) {
                            String reqId = request != null ? request.requestId : "unknown";
                            String type = request != null ? request.taskType : "unknown";
                            double time = request != null ? request.timestamp : 0;
                            response = TaskResponse.error(reqId, type, time, e.getMessage());
                        }

                        if (response != null) {
                            try {
                                String jsonResponse = mapper.writeValueAsString(response);
                                // IMPORTANT: Synchronize output to ensure raw wire JSON is perfectly discrete
                                synchronized (out) {
                                    out.println(jsonResponse);
                                }
                            } catch (Exception ex) {
                                ex.printStackTrace();
                            }
                        }
                    });
                }
            } catch (IOException e) {
                // Connection closed
            }
        }
    }
}
