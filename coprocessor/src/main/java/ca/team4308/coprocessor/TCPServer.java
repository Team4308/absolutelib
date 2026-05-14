package ca.team4308.coprocessor;

import ca.team4308.absolutelib.math.trajectories.network.ConfigurationPacket;
import ca.team4308.absolutelib.math.trajectories.network.LossyDataPacket;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryRequest;
import ca.team4308.absolutelib.math.trajectories.network.TrajectoryResponse;
import ca.team4308.absolutelib.math.trajectories.impl.Pose3d;
import ca.team4308.absolutelib.math.trajectories.impl.Rotation3d;
import ca.team4308.absolutelib.math.trajectories.TrajectoryResult;
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
import java.util.concurrent.ConcurrentHashMap;

public class TCPServer implements Runnable {

    private final ObjectMapper mapper = new ObjectMapper();
    private final TrajectoryWrapper solverWrapper;
    
    // For monitoring from dashboard
    public final AtomicReference<TrajectoryRequest> latestRequest = new AtomicReference<>(null);
    public final AtomicReference<TrajectoryResponse> latestResponse = new AtomicReference<>(null);
    public final AtomicReference<ConfigurationPacket> latestConfig = new AtomicReference<>(null);
    public final AtomicReference<LossyDataPacket> latestLossyPacket = new AtomicReference<>(null);
    public final AtomicReference<ca.team4308.absolutelib.math.trajectories.network.FullTelemetryPacket> latestFullTelemetry = new AtomicReference<>(null);
    private long telemetrySequence = 0;
    public final AtomicLong lastSolverTimeMs = new AtomicLong(0L);
    public final AtomicReference<Boolean> isConnected = new AtomicReference<>(false);
    public final AtomicLong lastActivityMs = new AtomicLong(0L);
    public static final long CONNECTION_TIMEOUT_MS = 2000L;
    
    public final AtomicLong totalRequests = new AtomicLong(0L);
    public final AtomicLong droppedPackets = new AtomicLong(0L);
    public final AtomicLong incomingPackets = new AtomicLong(0L);
    public final AtomicLong outgoingPackets = new AtomicLong(0L);
    public final AtomicReference<Double> batteryLevel = new AtomicReference<>(100.0);
    
    // Configuration tracking
    public final AtomicReference<Integer> activeConfigVersionId = new AtomicReference<>(0);
    public final LatencyFilter latencyFilter = new LatencyFilter(16);
    
    // Client connection pool
    private final ConcurrentHashMap<String, Long> clientConnectTimes = new ConcurrentHashMap<>();
    
    // Pre-allocated buffers for zero-allocation loop

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
                                System.out.println("Connection timeout detected.");
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
                clientSocket.setTcpNoDelay(true);
                String clientAddr = clientSocket.getInetAddress().toString();
                System.out.println("Client connected: " + clientAddr);
                isConnected.set(true);
                lastActivityMs.set(System.currentTimeMillis());
                clientConnectTimes.put(clientAddr, System.currentTimeMillis());

                Thread clientHandler = new Thread(() -> handleClient(clientSocket, clientAddr));
                clientHandler.setDaemon(true);
                clientHandler.start();
            }
        } catch (Exception e) {
            System.err.println("Could not listen on port " + Config.TCP_PORT);
            e.printStackTrace();
        }
    }

    private void handleClient(Socket clientSocket, String clientAddr) {
        ByteBuffer byteBuffer = ByteBuffer.allocate(Math.max(TrajectoryRequest.BINARY_SIZE, ConfigurationPacket.BINARY_SIZE));
        byteBuffer.order(ByteOrder.LITTLE_ENDIAN);

        try (DataInputStream dataIn = new DataInputStream(clientSocket.getInputStream());
             OutputStream rawOut = clientSocket.getOutputStream()) {

            while (!Thread.currentThread().isInterrupted()) {
                int firstByte = dataIn.read();
                if (firstByte == -1) break;

                long start = System.currentTimeMillis();
                TrajectoryRequest request = null;
                ConfigurationPacket config = null;

                try {
                    if (firstByte == TrajectoryRequest.MAGIC_BYTE) {
                        byte[] buf = new byte[TrajectoryRequest.BINARY_SIZE];
                        dataIn.readFully(buf);
                        byteBuffer.clear();
                        byteBuffer.put(buf);
                        byteBuffer.flip();
                        request = TrajectoryRequest.fromBuffer(byteBuffer);
                    } else if (firstByte == ConfigurationPacket.MAGIC_BYTE) {
                        byte[] buf = new byte[ConfigurationPacket.BINARY_SIZE];
                        dataIn.readFully(buf);
                        byteBuffer.clear();
                        byteBuffer.put(buf);
                        byteBuffer.flip();
                        config = ConfigurationPacket.fromBuffer(byteBuffer);
                    } else if (firstByte == '{') {
                        ByteArrayOutputStream baos = new ByteArrayOutputStream();
                        baos.write(firstByte);
                        int b;
                        while ((b = dataIn.read()) != -1 && b != '\n') {
                            baos.write(b);
                        }
                        
                        byte[] jsonBytes = baos.toByteArray();
                        try {
                            config = mapper.readValue(jsonBytes, ConfigurationPacket.class);
                        } catch (Exception e1) {
                            try {
                                request = mapper.readValue(jsonBytes, TrajectoryRequest.class);
                            } catch (Exception e2) {
                                System.err.println("Failed to parse JSON: " + e2.getMessage());
                                droppedPackets.incrementAndGet();
                                continue;
                            }
                        }
                    } else {
                        continue;
                    }

                    lastActivityMs.set(System.currentTimeMillis());

                    if (config != null) {
                        incomingPackets.incrementAndGet();
                        latestConfig.set(config);
                        activeConfigVersionId.set(config.configVersionId);
                        
                        solverWrapper.updateConfiguration(config);
                        System.out.println("Configuration updated: version " + config.configVersionId);
                        
                        java.util.Map<String, Object> ackMap = new java.util.HashMap<>();
                        ackMap.put("ack", "config_received");
                        ackMap.put("version", config.configVersionId);
                        byte[] ackJson = mapper.writeValueAsBytes(ackMap);
                        rawOut.write(ackJson);
                        rawOut.write('\n');
                        rawOut.flush();
                        outgoingPackets.incrementAndGet();
                    }

                    // Handle trajectory request
                    if (request != null) {
                        incomingPackets.incrementAndGet();
                        totalRequests.incrementAndGet();
                        latestRequest.set(request);
                        if (request.battery > 0) {
                            setBattery(request.battery);
                        }

                        TrajectoryResponse response = solverWrapper.solve(request);
                        latestResponse.set(response);

                        TrajectoryResult trajResult = solverWrapper.getShooterSystem().getLastTrajectoryResult();

                        byteBuffer.clear();
                        response.toBuffer(byteBuffer);
                        rawOut.write(byteBuffer.array(), 0, TrajectoryResponse.BINARY_SIZE);
                        outgoingPackets.incrementAndGet();
                        rawOut.flush();

                        // Optional lossy flight path packet (non-critical)
                        LossyDataPacket lossyPacket = buildLossyPacket(trajResult);
                        if (lossyPacket != null && !lossyPacket.flightPath.isEmpty()) {
                            latestLossyPacket.set(lossyPacket);
                            ByteBuffer lossyBuffer = ByteBuffer.allocate(LossyDataPacket.BINARY_SIZE);
                            lossyBuffer.order(ByteOrder.LITTLE_ENDIAN);
                            lossyPacket.toBuffer(lossyBuffer);
                            rawOut.write(lossyBuffer.array(), 0, LossyDataPacket.BINARY_SIZE);
                            outgoingPackets.incrementAndGet();
                            rawOut.flush();
                        }

                        // Generate full telemetry for UDP and WebSockets
                        latestFullTelemetry.set(buildFullTelemetry(request, response, trajResult));

                        long elapsed = System.currentTimeMillis() - start;
                        lastSolverTimeMs.set(elapsed);
                        latencyFilter.addSample(elapsed);

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
            System.out.println("Client disconnected: " + clientAddr);
            isConnected.set(false);
            lastActivityMs.set(0L);
            clientConnectTimes.remove(clientAddr);
            try {
                clientSocket.close();
            } catch (Exception e) {
            }
        }
    }

    private LossyDataPacket buildLossyPacket(TrajectoryResult trajResult) {
        if (trajResult == null || !trajResult.isSuccess()) {
            return null;
        }

        java.util.List<edu.wpi.first.math.geometry.Pose3d> flightPath = trajResult.getFlightPath();
        if (flightPath == null || flightPath.isEmpty()) {
            return null;
        }

        LossyDataPacket packet = new LossyDataPacket();
        int sampleRate = Math.max(1, flightPath.size() / LossyDataPacket.MAX_POINTS);
        for (int i = 0; i < flightPath.size(); i += sampleRate) {
            if (packet.flightPath.size() >= LossyDataPacket.MAX_POINTS) {
                break;
            }
            edu.wpi.first.math.geometry.Pose3d pose = flightPath.get(i);
            packet.flightPath.add(new Pose3d(pose.getX(), pose.getY(), pose.getZ(), new Rotation3d()));
        }

        if (!flightPath.isEmpty() && packet.flightPath.size() < LossyDataPacket.MAX_POINTS) {
            edu.wpi.first.math.geometry.Pose3d last = flightPath.get(flightPath.size() - 1);
            packet.flightPath.add(new Pose3d(last.getX(), last.getY(), last.getZ(), new Rotation3d()));
        }

        return packet;
    }

    private ca.team4308.absolutelib.math.trajectories.network.FullTelemetryPacket buildFullTelemetry(
            TrajectoryRequest request, TrajectoryResponse response, ca.team4308.absolutelib.math.trajectories.TrajectoryResult result) {
        
        ca.team4308.absolutelib.math.trajectories.network.FullTelemetryPacket packet = new ca.team4308.absolutelib.math.trajectories.network.FullTelemetryPacket();
        packet.timestamp = System.currentTimeMillis() / 1000.0;
        packet.sequence = telemetrySequence++;
        
        // Solution
        packet.status = response.status;
        packet.pitchDeg = response.pitchDegrees;
        packet.yawDeg = response.yawDegrees;
        packet.rpm = response.rpm;
        packet.exitVelocityMps = response.rpm > 0 ? (response.rpm * 0.1016 * Math.PI / 60.0) : 0; // Rough estimate or use sim exit vel
        packet.confidence = response.confidence;
        
        // Path
        packet.setFlightPath(result.getFlightPath());
        
        // Flywheel Sim
        if (result.hasFlywheelData()) {
            ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelSimulator.SimulationResult sim = result.getFlywheelSimulation();
            packet.fwWheelRpm = sim.requiredWheelRpm;
            packet.fwMotorRpm = sim.requiredMotorRpm;
            packet.fwMotorPower = sim.motorPowerPercent;
            packet.fwSpinUpSec = sim.spinUpTimeSeconds;
            packet.fwCurrentAmps = sim.currentDrawAmps;
            packet.fwStoredJoules = sim.storedEnergyJoules;
            packet.fwContactMs = sim.contactTimeMs;
            packet.fwBallSpinRpm = sim.ballSpinRpm;
            packet.fwSlipRatio = sim.slipRatio;
            packet.fwEfficiency = sim.energyTransferEfficiency;
            packet.fwAchievable = sim.isAchievable;
            packet.fwLimitingFactor = sim.limitingFactor;
            packet.exitVelocityMps = sim.exitVelocityMps; // Better exit velocity from sim
        }
        
        // Metrics
        packet.metTof = result.getTimeOfFlightSeconds();
        packet.metMaxHeight = result.getMaxHeightMeters();
        packet.metMarginError = result.getMarginOfErrorMeters();
        packet.metDistance = result.getDistanceToTargetMeters();
        packet.metHeightDiff = result.getHeightDifferenceMeters();
        
        // Trace
        packet.trMode = result.getSolveModeUsed().name();
        packet.trTimeMs = result.getComputationTimeMs();
        packet.trIterations = result.getIterations();
        
        ca.team4308.absolutelib.math.trajectories.SolveDebugInfo debug = result.getDebugInfo();
        if (debug != null) {
            packet.trTotalTested = debug.getTotalTested();
            packet.trAccepted = debug.getAcceptedCount();
            packet.trRejCollision = debug.getRejectedCollisionCount();
            packet.trRejArcTooLow = debug.getRejectedArcTooLowCount();
            packet.trRejClearance = debug.getRejectedClearanceCount();
            packet.trRejMiss = debug.getRejectedMissCount();
            packet.trRejFlyover = debug.getRejectedFlyoverCount();
        }
        
        // Discrete
        if (result.hasDiscreteSolution()) {
            ca.team4308.absolutelib.math.trajectories.TrajectoryResult.DiscreteShot ds = result.getDiscreteSolution();
            packet.dsValid = true;
            packet.dsRpm = ds.rpmValue;
            packet.dsPitchDeg = ds.pitchAngleDegrees;
            packet.dsRpmTicks = ds.rpmTicks;
            packet.dsAngleTicks = ds.angleTicks;
            packet.dsScore = ds.score;
        }
        
        // Input
        packet.inRobotX = request.robotX;
        packet.inRobotY = request.robotY;
        packet.inRobotZ = request.robotZ;
        packet.inTargetX = request.targetX;
        packet.inTargetY = request.targetY;
        packet.inTargetZ = request.targetZ;
        packet.inVx = request.vxMps;
        packet.inVy = request.vyMps;
        
        return packet;
    }

    public int getActiveConfigVersionId() {
        Integer ver = activeConfigVersionId.get();
        return ver != null ? ver : 0;
    }

    public double getFilteredLatencyMs() {
        return latencyFilter.getLastFilteredValue();
    }
}
