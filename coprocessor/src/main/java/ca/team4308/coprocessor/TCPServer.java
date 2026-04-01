package ca.team4308.coprocessor;

import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.DataInputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class TCPServer implements Runnable {

    private final ObjectMapper mapper = new ObjectMapper();
    private final TrajectoryWrapper solverWrapper;
    
    // For monitoring from dashboard
    public final AtomicReference<TrajectoryRequest> latestRequest = new AtomicReference<>(null);
    public final AtomicReference<TrajectoryResponse> latestResponse = new AtomicReference<>(null);
    public final AtomicLong lastSolverTimeMs = new AtomicLong(0L);
    public final AtomicReference<Boolean> isConnected = new AtomicReference<>(false);
    public final AtomicLong lastActivityMs = new AtomicLong(0L);
    public static final long CONNECTION_TIMEOUT_MS = 2000L;
    
    public final AtomicLong totalRequests = new AtomicLong(0L);
    public final AtomicLong droppedPackets = new AtomicLong(0L);
    public final AtomicLong incomingPackets = new AtomicLong(0L);
    public final AtomicLong outgoingPackets = new AtomicLong(0L);
    public final AtomicReference<Double> batteryLevel = new AtomicReference<>(100.0);

    public void setBattery(double incomingBattery) {
        if (Double.isNaN(incomingBattery) || Double.isInfinite(incomingBattery)) {
            return;
        }

        double normalized = incomingBattery;

        // Compatibility with common reporting formats:
        // - 0.0..1.0 (fraction) -> 0..100%
        // - 0..20 (voltage) -> map to 0..100% (assume 12V nominal)
        // - >1000 e.g. 1005 from scaled integer form -> divide by 10.
        if (normalized > 0 && normalized <= 1.5) {
            normalized = normalized * 100.0;
        } else if (normalized > 1.5 && normalized <= 20.0) {
            normalized = normalized / 12.0 * 100.0;
        } else if (normalized > 1000.0) {
            normalized = normalized / 10.0;
        }

        normalized = Math.min(100.0, Math.max(0.0, normalized));
        batteryLevel.set(normalized);
    }
    
    public TCPServer(TrajectoryWrapper solverWrapper) {
        this.solverWrapper = solverWrapper;
    }

    @Override
    public void run() {
        try (ServerSocket serverSocket = new ServerSocket(Config.TCP_PORT)) {
            System.out.println("TCP Server listening on port " + Config.TCP_PORT);

            Thread timeoutThread = new Thread(() -> {
                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        if (isConnected.get()) {
                            long last = lastActivityMs.get();
                            if (last > 0 && System.currentTimeMillis() - last > CONNECTION_TIMEOUT_MS) {
                                isConnected.set(false);
                            }
                        }
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            });
            timeoutThread.setDaemon(true);
            timeoutThread.start();
            
            while (!Thread.currentThread().isInterrupted()) {
                Socket clientSocket = serverSocket.accept();
                clientSocket.setTcpNoDelay(true); // Disable Nagle's algorithm for lowest latency
                System.out.println("Client connected: " + clientSocket.getInetAddress());
                isConnected.set(true);
                lastActivityMs.set(System.currentTimeMillis());

                try (DataInputStream dataIn = new DataInputStream(clientSocket.getInputStream());
                     OutputStream rawOut = clientSocket.getOutputStream()) {

                    while (!Thread.currentThread().isInterrupted()) {
                        int firstByte = dataIn.read();
                        if (firstByte == -1) break;

                        long start = System.currentTimeMillis();
                        TrajectoryRequest request = null;

                        try {
                            if (firstByte == TrajectoryRequest.MAGIC_BYTE) {
                                byte[] buf = new byte[TrajectoryRequest.BINARY_SIZE];
                                dataIn.readFully(buf);
                                ByteBuffer bb = ByteBuffer.wrap(buf).order(ByteOrder.LITTLE_ENDIAN);
                                request = TrajectoryRequest.fromBuffer(bb);
                            } else if (firstByte == '{') {
                                ByteArrayOutputStream baos = new ByteArrayOutputStream();
                                baos.write(firstByte);
                                while (true) {
                                    int b = dataIn.read();
                                    if (b == -1 || b == '\n') break;
                                    baos.write(b);
                                }
                                request = mapper.readValue(baos.toByteArray(), TrajectoryRequest.class);
                            } else {
                                continue;
                            }
                            
                            if (request != null) {
                                lastActivityMs.set(System.currentTimeMillis());
                                incomingPackets.incrementAndGet();
                                totalRequests.incrementAndGet();
                                latestRequest.set(request);
                                if (request.battery > 0) {
                                    setBattery(request.battery);
                                }

                                TrajectoryResponse response = solverWrapper.solve(request);
                                latestResponse.set(response);

                                if (firstByte == TrajectoryRequest.MAGIC_BYTE) {
                                    ByteBuffer outBuf = ByteBuffer.allocate(TrajectoryResponse.BINARY_SIZE).order(ByteOrder.LITTLE_ENDIAN);
                                    response.toBuffer(outBuf);
                                    rawOut.write(outBuf.array());
                                    outgoingPackets.incrementAndGet();
                                } else {
                                    byte[] json = mapper.writeValueAsBytes(response);
                                    rawOut.write(json);
                                    rawOut.write('\n');
                                    outgoingPackets.incrementAndGet();
                                }
                                rawOut.flush();

                                lastSolverTimeMs.set(System.currentTimeMillis() - start);

                                if (Config.LOG_TO_FILE) {
                                    ReplayLogger.log(request, response);
                                }
                            }
                        } catch (Exception e) {
                            System.err.println("Error processing request: " + e.getMessage());
                            droppedPackets.incrementAndGet();
                        }
                    }
                } catch (Exception e) {
                    System.err.println("Client handler exception: " + e.getMessage());
                } finally {
                    System.out.println("Client disconnected.");
                    isConnected.set(false);
                    lastActivityMs.set(0L);
                }
            }
        } catch (Exception e) {
            System.err.println("Could not listen on port " + Config.TCP_PORT);
            e.printStackTrace();
        }
    }
}
