package frc.robot.subsystems;

import ca.team4308.absolutelib.math.trajectories.network.LossyDataPacket;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.DataInputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.io.ByteArrayOutputStream;

public class CoprocessorClient implements Runnable {
    // lowkey this code is pretty complex but basicly allows you to throw anything that is to heavy for the rio into a coprocessor and get the results back, it supports both json and binary protocols, but binary is much faster and more efficient, so use that if you can
    private final String host;
    private final int port;
    private final boolean useBinaryProtocol;
    private final ObjectMapper mapper = new ObjectMapper();
    private final AtomicReference<Long> lastMs = new AtomicReference<>(System.currentTimeMillis());
    private final AtomicReference<TrajectoryRequest> currentRequest = new AtomicReference<>(null);
    private final AtomicReference<TrajectoryResponse> latestResponse = new AtomicReference<>(null);
    private final AtomicReference<List<edu.wpi.first.math.geometry.Pose3d>> latestFlightPath = new AtomicReference<>(List.of());
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

    public List<edu.wpi.first.math.geometry.Pose3d> getLatestFlightPath() {
        return latestFlightPath.get();
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
                        System.out.println("getLatestResponse: " + String.valueOf(getLatestResponse() != null ? getLatestResponse().toString() : "null"));
                        System.out.println("currentRequest: " + String.valueOf(currentRequest.get() != null ? currentRequest.get().toString() : "null"));
                        System.out.println("isConnected: "+ String.valueOf(isConnected.get()));
                        System.out.println("useBinaryProtocol: "+ String.valueOf(useBinaryProtocol));
                        System.out.println("Ping (MS): " + String.valueOf(System.currentTimeMillis() - lastMs.get()));
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
                                    lastMs.set(System.currentTimeMillis());

                                    // Optional lossy flight path packet (non-critical)
                                    if (dataIn.available() >= LossyDataPacket.BINARY_SIZE) {
                                        int lossyMagic = dataIn.read();
                                        if (lossyMagic == LossyDataPacket.MAGIC_BYTE) {
                                            byte[] lossyBuf = new byte[LossyDataPacket.BINARY_SIZE - 1];
                                            dataIn.readFully(lossyBuf);
                                            ByteBuffer lossyBb = ByteBuffer.wrap(lossyBuf).order(ByteOrder.LITTLE_ENDIAN);
                                            LossyDataPacket packet = LossyDataPacket.fromBuffer(lossyBb);
                                            latestFlightPath.set(convertToWpiPoses(packet));
                                        }
                                    }
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

    private List<edu.wpi.first.math.geometry.Pose3d> convertToWpiPoses(LossyDataPacket packet) {
        if (packet == null || packet.flightPath == null || packet.flightPath.isEmpty()) {
            return List.of();
        }

        List<edu.wpi.first.math.geometry.Pose3d> poses = new ArrayList<>(packet.flightPath.size());
        for (ca.team4308.absolutelib.math.trajectories.impl.Pose3d pose : packet.flightPath) {
            poses.add(new edu.wpi.first.math.geometry.Pose3d(
                    pose.getTranslation().x,
                    pose.getTranslation().y,
                    pose.getTranslation().z,
                    new edu.wpi.first.math.geometry.Rotation3d()));
        }
        return poses;
    }
}
