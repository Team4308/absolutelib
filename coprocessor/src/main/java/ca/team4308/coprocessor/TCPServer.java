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
    
    public final AtomicLong totalRequests = new AtomicLong(0L);
    public final AtomicLong droppedPackets = new AtomicLong(0L);
    
    public TCPServer(TrajectoryWrapper solverWrapper) {
        this.solverWrapper = solverWrapper;
    }

    @Override
    public void run() {
        try (ServerSocket serverSocket = new ServerSocket(Config.TCP_PORT)) {
            System.out.println("TCP Server listening on port " + Config.TCP_PORT);
            
            while (!Thread.currentThread().isInterrupted()) {
                Socket clientSocket = serverSocket.accept();
                clientSocket.setTcpNoDelay(true); // Disable Nagle's algorithm for lowest latency
                System.out.println("Client connected: " + clientSocket.getInetAddress());
                isConnected.set(true);

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
                                totalRequests.incrementAndGet();
                                latestRequest.set(request);

                                TrajectoryResponse response = solverWrapper.solve(request);
                                latestResponse.set(response);

                                if (firstByte == TrajectoryRequest.MAGIC_BYTE) {
                                    ByteBuffer outBuf = ByteBuffer.allocate(TrajectoryResponse.BINARY_SIZE).order(ByteOrder.LITTLE_ENDIAN);
                                    response.toBuffer(outBuf);
                                    rawOut.write(outBuf.array());
                                } else {
                                    byte[] json = mapper.writeValueAsBytes(response);
                                    rawOut.write(json);
                                    rawOut.write('\n');
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
                }
            }
        } catch (Exception e) {
            System.err.println("Could not listen on port " + Config.TCP_PORT);
            e.printStackTrace();
        }
    }
}
