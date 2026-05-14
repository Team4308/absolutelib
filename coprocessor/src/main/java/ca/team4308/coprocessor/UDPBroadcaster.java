package ca.team4308.coprocessor;

import ca.team4308.absolutelib.math.trajectories.network.FullTelemetryPacket;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class UDPBroadcaster implements Runnable {

    private final TCPServer tcpServer;
    private final ObjectMapper mapper = new ObjectMapper();
    private long lastSequenceSent = -1;

    public UDPBroadcaster(TCPServer tcpServer) {
        this.tcpServer = tcpServer;
    }

    @Override
    public void run() {
        if (!Config.UDP_ENABLED) {
            System.out.println("UDP Broadcaster disabled in config.");
            return;
        }

        System.out.println("UDP Broadcaster starting on " + Config.UDP_BROADCAST_ADDR + ":" + Config.UDP_PORT);

        try (DatagramSocket socket = new DatagramSocket()) {
            socket.setBroadcast(true);
            InetAddress address = InetAddress.getByName(Config.UDP_BROADCAST_ADDR);

            while (!Thread.currentThread().isInterrupted()) {
                FullTelemetryPacket packet = tcpServer.latestFullTelemetry.get();

                if (packet != null && packet.sequence != lastSequenceSent) {
                    try {
                        byte[] data = mapper.writeValueAsBytes(packet);
                        
                        if (data.length > 1472) {
                            // If too large, we might want to trim path points, 
                            // but for now just warn and send anyway (IP fragmentation)
                            // System.err.println("Warning: UDP packet size " + data.length + " exceeds MTU");
                        }

                        DatagramPacket datagram = new DatagramPacket(data, data.length, address, Config.UDP_PORT);
                        socket.send(datagram);
                        lastSequenceSent = packet.sequence;
                        tcpServer.outgoingPackets.incrementAndGet();
                    } catch (Exception e) {
                        System.err.println("Error broadcasting UDP packet: " + e.getMessage());
                    }
                }

                // Run at ~50Hz
                Thread.sleep(20);
            }
        } catch (Exception e) {
            System.err.println("UDP Broadcaster thread crashed: " + e.getMessage());
            e.printStackTrace();
        }
    }
}
