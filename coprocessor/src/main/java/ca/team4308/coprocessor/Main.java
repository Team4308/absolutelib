package ca.team4308.coprocessor;

import ca.team4308.absolutelib.math.trajectories.TrajectoryResult;
import ca.team4308.absolutelib.network.task.server.TaskRegistry;
import ca.team4308.absolutelib.network.task.server.TaskServer;

public class Main {
    public static void main(String[] args) {
        try {
            mainInternal(args);
        } catch (Throwable ex) {
            System.err.println("[MAIN CATCH] Unexpected exception in main");
            System.err.println("[MAIN CATCH] Type: " + ex.getClass().getName());
            ex.printStackTrace(System.err);
            System.exit(1);
        }
    }

    private static void mainInternal(String[] args) {
        if (args.length == 2 && args[0].equals("--replay")) {
            ReplayLogger.replay(args[1]);
            return;
        }

        System.out.println("=========================================");
        System.out.println("Starting Trajectory Coprocessor");
        System.out.println("Team Number: " + Config.TEAM_NUMBER);
        System.out.println("Simulation: " + Config.IS_SIMULATION);
        System.out.println("=========================================");

        if (Config.LOG_TO_FILE) {
            System.out.println("Logging enabled to: " + Config.LOG_FILE_PATH);
            ReplayLogger.start(Config.LOG_FILE_PATH);
        }

        TrajectoryWrapper wrapper = new TrajectoryWrapper();
        TCPServer tcpServer = new TCPServer(wrapper);
    TaskRegistry registry = new TaskRegistry();
    TaskServer taskServer = new TaskServer(5802, registry, 4); // 4 concurrent worker threads
    new WebServer(tcpServer, taskServer);
        NT4Publisher nt4Publisher = null;
        try {
            nt4Publisher = new NT4Publisher();
        } catch (Throwable ex) {
            System.out.println("Skipping NT4Publisher (ntcorejni library not available)");
        }

        Thread tcpThread = new Thread(tcpServer);
        tcpThread.start();

    // Start the generalized task offloading server on port 5802
    registry.register("TRAJECTORY_SOLVE", new TrajectoryTaskHandler(wrapper, tcpServer));
    Thread taskThread = new Thread(taskServer);
    taskThread.start();

        System.out.println("System initialized entirely.");

        // Main thread loop to update NT4
        while (true) {
            try {
                Thread.sleep(20); // 50Hz update loop to NT4
                TrajectoryResult trajResult = wrapper.getShooterSystem().getLastTrajectoryResult();
                if (nt4Publisher != null) {
                    nt4Publisher.update(
                        tcpServer.latestRequest.get(),
                        tcpServer.latestResponse.get(),
                        tcpServer.lastSolverTimeMs.get(),
                        trajResult,
                        tcpServer.isConnected.get(),
                        tcpServer.lastSolverTimeMs.get(),
                        tcpServer.totalRequests.get(),
                        tcpServer.droppedPackets.get(),
                        tcpServer
                    );
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
        }
    }
}
