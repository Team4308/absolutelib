package frc.robot.subsystems;

import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.DataInputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.concurrent.atomic.AtomicReference;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.io.ByteArrayOutputStream;

public class CoprocessorClient implements Runnable {
    // lowkey this code is pretty complex but basicly allows you to throw anything that is to heavy for the rio into a coprocessor and get the results back, it supports both json and binary protocols, but binary is much faster and more efficient, so use that if you can
    private final String host;
    private final int port;
    private final boolean useBinaryProtocol;
    private final ObjectMapper mapper = new ObjectMapper();

    private final AtomicReference<TrajectoryRequest> currentRequest = new AtomicReference<>(null);
    private final AtomicReference<TrajectoryResponse> latestResponse = new AtomicReference<>(null);
    private final AtomicReference<Boolean> isConnected = new AtomicReference<>(false);

    public CoprocessorClient(String host, int port, boolean useBinaryProtocol) {
        this.host = host;
        this.port = port;
        this.useBinaryProtocol = useBinaryProtocol;
    }

    public void setRequest(TrajectoryRequest request) {
        currentRequest.set(request);
    }

    public TrajectoryResponse getLatestResponse() {
        return latestResponse.get();
    }

    public boolean isConnected() {
        return isConnected.get();
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                System.out.println("CoprocessorClient: Connecting to " + host + ":" + port);
                try (Socket socket = new Socket(host, port);
                     OutputStream rawOut = socket.getOutputStream();
                     DataInputStream dataIn = new DataInputStream(socket.getInputStream())) {

                    socket.setTcpNoDelay(true);
                    isConnected.set(true);
                    System.out.println("CoprocessorClient: Connected.");

                    while (isConnected.get() && !Thread.currentThread().isInterrupted()) {
                        TrajectoryRequest req = currentRequest.get();
                        if (req != null) {
                            if (useBinaryProtocol) {
                                ByteBuffer outBuf = ByteBuffer.allocate(TrajectoryRequest.BINARY_SIZE + 1).order(ByteOrder.LITTLE_ENDIAN);
                                outBuf.put(TrajectoryRequest.MAGIC_BYTE);
                                req.toBuffer(outBuf);
                                rawOut.write(outBuf.array());
                                rawOut.flush();

                                int check = dataIn.read();
                                if (check == -1) break;
                                if (check == TrajectoryResponse.MAGIC_BYTE) {
                                    byte[] inBuf = new byte[TrajectoryResponse.BINARY_SIZE - 1];
                                    dataIn.readFully(inBuf);
                                    ByteBuffer bb = ByteBuffer.wrap(inBuf).order(ByteOrder.LITTLE_ENDIAN);
                                    TrajectoryResponse res = TrajectoryResponse.fromBuffer(bb);
                                    latestResponse.set(res);
                                }
                            } else {
                                String jsonReq = mapper.writeValueAsString(req) + "\n";
                                rawOut.write(jsonReq.getBytes());
                                rawOut.flush();

                                ByteArrayOutputStream baos = new ByteArrayOutputStream();
                                while (true) {
                                    int b = dataIn.read();
                                    if (b == -1 || b == '\n') break;
                                    baos.write(b);
                                }
                                /// Some times??
                                if (baos.size() == 0) break; // closed
                                TrajectoryResponse res = mapper.readValue(baos.toByteArray(), TrajectoryResponse.class);
                                latestResponse.set(res);
                            }
                        }
                        Thread.sleep(10); // Throttle loop to ~100 Hz
                    }

                } catch (Exception e) {
                    System.err.println("CoprocessorClient Connection Error: " + e.getMessage());
                } finally {
                    isConnected.set(false);
                }

                // TODO: Change Value
                Thread.sleep(1000);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
