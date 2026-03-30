package ca.team4308.coprocessor;

import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.*;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class ReplayLogger {
    private static final ObjectMapper mapper = new ObjectMapper();
    private static final BlockingQueue<LogEntry> queue = new LinkedBlockingQueue<>(1000);
    private static Thread loggerThread;

    static class LogEntry {
        TrajectoryRequest req;
        TrajectoryResponse res;
        LogEntry(TrajectoryRequest req, TrajectoryResponse res) {
            this.req = req;
            this.res = res;
        }
    }

    public static void start(String filePath) {
        loggerThread = new Thread(() -> {
            try (PrintWriter out = new PrintWriter(new FileWriter(filePath, true))) {
                while (!Thread.currentThread().isInterrupted()) {
                    LogEntry entry = queue.take();
                    String reqStr = mapper.writeValueAsString(entry.req);
                    String resStr = mapper.writeValueAsString(entry.res);
                    out.println(reqStr + "|" + resStr);
                    out.flush();
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } catch (Exception e) {
                e.printStackTrace();
            }
        });
        loggerThread.setDaemon(true);
        loggerThread.setName("ReplayLogger");
        loggerThread.start();
    }

    public static void log(TrajectoryRequest req, TrajectoryResponse res) {
        if (loggerThread != null && loggerThread.isAlive()) {
            queue.offer(new LogEntry(req, res)); // Non-blocking, drops if full
        }
    }

    public static void replay(String filePath) {
        System.out.println("Starting Replay Diagnostic from " + filePath);
        TrajectoryWrapper wrapper = new TrajectoryWrapper();
        
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            int count = 0;
            while ((line = br.readLine()) != null) {
                String[] parts = line.split("\\|");
                if (parts.length == 2) {
                    TrajectoryRequest req = mapper.readValue(parts[0], TrajectoryRequest.class);
                    TrajectoryResponse oldRes = mapper.readValue(parts[1], TrajectoryResponse.class);
                    
                    TrajectoryResponse newRes = wrapper.solve(req);
                    
                    System.out.println("Req #" + (++count) + 
                        " | Pitch Diff: " + String.format("%.3f", Math.abs(newRes.pitchDegrees - oldRes.pitchDegrees)) + 
                        " | Yaw Diff: " + String.format("%.3f", Math.abs(newRes.yawDegrees - oldRes.yawDegrees)));
                }
            }
            System.out.println("Replay complete. Processed " + count + " historical requests.");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
