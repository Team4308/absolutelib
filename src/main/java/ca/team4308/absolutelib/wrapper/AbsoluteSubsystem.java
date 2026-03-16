package ca.team4308.absolutelib.wrapper;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Extends SubsystemBase to provide logging capabilities and more helper
 * methods.
 */
public abstract class AbsoluteSubsystem extends SubsystemBase {

    /**
     * Provide a Sendable for telemetry/logging; may return null if unused.
     * @return the Sendable associated with this subsystem
     */
    public abstract Sendable log();

    private volatile LoggingBackend backend;
    private final Map<String, Long> lastLogMs = new HashMap<>();
    private final Set<String> loggedOnce = new HashSet<>();

    /**
     * Constructs a new AbsoluteSubsystem.
     */
    public AbsoluteSubsystem() {
        this.backend = new DriverStationBackend();
    }

    /*
     * Returns the base log channel for this subsystem.
     */
    private static boolean loggerAvailable = false;
    private static boolean loggerChecked = false;

    @SuppressWarnings("unused")
    private static boolean isLoggerAvailable() {
        if (!loggerChecked) {
            loggerChecked = true;
            try {
                Logger.recordOutput("/Test/Post", true);
                NetworkTableInstance defaultInst = NetworkTableInstance.getDefault();
                if (true) { // Need to fix this ltr using network tables? so deadcode
                    loggerAvailable = true;
                } else {
                    loggerAvailable = false;
                }
            } catch (Exception | NoClassDefFoundError e) {
                loggerAvailable = false;
            }
        }
        return loggerAvailable;
    }

    /**
     * Gets the base log channel for this subsystem.
     * @return the log channel string
     */
    protected String getLogChannelBase() {
        String n = getName();
        if (n == null || n.isEmpty()) {
            n = getClass().getSimpleName();
        }
        return "/subsystems/" + n;
    }

    /**
     * Record an output value for logging/telemetry. Uses AdvantageKit's Logger
     * if available, otherwise falls back to SmartDashboard.
     * @param name Key for the output
     * @param value The value to record
     */
    protected void recordOutput(String name, Object value) {
        String fullKey = getLogChannelBase() + "/" + name;
        if (isLoggerAvailable()) {
            recordToLogger(fullKey, value);
        } else {
            recordToSmartDashboard(fullKey, value);
        }
    }

    private void recordToLogger(String key, Object value) {
        try {
            if (value instanceof Double) {
                Logger.recordOutput(key, (Double) value);
            } else if (value instanceof Integer) {
                Logger.recordOutput(key, (Integer) value);
            } else if (value instanceof Boolean) {
                Logger.recordOutput(key, (Boolean) value);
            } else if (value instanceof String) {
                Logger.recordOutput(key, (String) value);
            } else if (value instanceof Sendable) {
                SmartDashboard.putData(key, (Sendable) value);
            } else if (value instanceof double[]) {
                Logger.recordOutput(key, (double[]) value);
            } else if (value instanceof Pose3d[]) {
                Logger.recordOutput(key, (Pose3d[]) value);
            } else if (value instanceof Pose2d[]) {
                Logger.recordOutput(key, (Pose2d[]) value);
            } else if (value instanceof Pose3d) {
                Logger.recordOutput(key, (Pose3d) value);
            } else if (value instanceof Pose2d) {
                Logger.recordOutput(key, (Pose2d) value);
            } else if (value != null) {
                Logger.recordOutput(key, value.toString());
            }
        } catch (Exception e) {
            recordToSmartDashboard(key, value);
        }
    }

    private void recordToSmartDashboard(String key, Object value) {
        if (value instanceof Double) {
            SmartDashboard.putNumber(key, (Double) value);
        } else if (value instanceof Integer) {
            SmartDashboard.putNumber(key, (Integer) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (Boolean) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(key, (String) value);
        } else if (value instanceof Sendable) {
            SmartDashboard.putData(key, (Sendable) value);
        } else if (value instanceof double[]) {
            SmartDashboard.putNumberArray(key, (double[]) value);
        } else if (value != null) {
            SmartDashboard.putString(key, value.toString());
        }
    }

    /**
     * Log info.
     * @param message the message to log
     */
    protected void logInfo(String message) {
        backend.info(getLogChannelBase(), message);
    }

    /**
     * Log a warning.
     * @param message the message to log
     */
    protected void logWarn(String message) {
        backend.warn(getLogChannelBase(), message);
    }

    /**
     * Log an error.
     * @param message the message to log
     */
    protected void logError(String message) {
        backend.error(getLogChannelBase(), message, null);
    }

    /**
     * Log an error with throwable
     * @param message the message to log
     * @param t the throwable
     */
    protected void logError(String message, Throwable t) {
        backend.error(getLogChannelBase(), message, t);
    }

    /**
     * Log a number value.
     * @param key the key
     * @param value the number
     */
    protected void logNumber(String key, double value) {
        backend.info(getLogChannelBase() + "/" + key, Double.toString(value));
    }

    /**
     * Log a boolean value.
     * @param key the key
     * @param value the boolean
     */
    protected void logBoolean(String key, boolean value) {
        backend.info(getLogChannelBase() + "/" + key, Boolean.toString(value));
    }

    /**
     * Log a message only the first time for the given key.
     * @param key the unique key for this log
     * @param message the message
     */
    protected void logOnce(String key, String message) {
        if (loggedOnce.add(key)) {
            logInfo("[once] " + message);
        }
    }

    /**
     * Throttled log: only logs if at least minIntervalMs since last time for
     * the key.
     * @param key the identifier
     * @param minIntervalMs the min interval
     * @param message the message
     */
    protected void logThrottle(String key, long minIntervalMs, String message) {
        long now = nowMs();
        Long prev = lastLogMs.get(key);
        if (prev == null || now - prev >= minIntervalMs) {
            lastLogMs.put(key, now);
            logInfo("[throttled] " + message);
        }
    }

    /**
     * Wrap a runnable and log any thrown exception with a label.
     * @param label the log label
     * @param body the runnable action
     */
    protected void runSafely(String label, Runnable body) {
        try {
            body.run();
        } catch (Throwable t) {
            logError("Exception in " + label, t);
        }
    }

    private static long nowMs() {
        return (long) (Timer.getFPGATimestamp() * 1_000.0);
    }

    @SuppressWarnings("unused")
    private String formatPrefix() {
        return "[" + getLogChannelBase() + "] ";
    }

    private static String stackTrace(Throwable t) {
        try (StringWriter sw = new StringWriter(); PrintWriter pw = new PrintWriter(sw)) {
            t.printStackTrace(pw);
            return sw.toString();
        } catch (Exception e) {
            return t.toString();

        }
    }

    /**
     * Interface for custom logging backends.
     */
    protected interface LoggingBackend {
        /**
         * Log an info message.
         * @param channel Log channel name
         * @param message Info message
         */
        void info(String channel, String message);

        /**
         * Log a warning message.
         * @param channel Log channel name
         * @param message Warning message
         */
        void warn(String channel, String message);

        /**
         * Log an error message.
         * @param channel Log channel name
         * @param message Error message
         * @param t Throwable (optional)
         */
        void error(String channel, String message, Throwable t);
    }

    /**
     * Fallback backend that reports via DriverStation only.
     */
    private static class DriverStationBackend implements LoggingBackend {

        @Override
        public void info(String channel, String message) {
            Logger.recordOutput(channel + "/info", message);
        }

        @Override
        public void warn(String channel, String message) {
            Logger.recordOutput(channel + "/warn", message);
        }

        @Override
        public void error(String channel, String message, Throwable t) {
            Logger.recordOutput(channel + "/error", message + (t != null ? ("\n" + stackTrace(t)) : ""));
            DriverStation.reportError(channel + " Has an error " + message + "Check /error for full error.", true);
        }
    }

    /**
     * Allows external code to supply a custom backend implementation.
     * @param custom the custom backend
     */
    public void setCustomLoggingBackend(LoggingBackend custom) {
        if (custom != null) {
            this.backend = custom;
    
        }}

    /**
     * Initializes the subsystem.
     */
    public void initialize() {
        onInitialize();
    }

    /**
     * Called during initialization.
     */
    protected void onInitialize() {
    }

    // Hoesntly unused but whatever have them for ltr
    /**
     * Called before periodic execution.
     */
    protected void onPrePeriodic() {
    }

    /**
     * Called after periodic execution.
     */
    protected void onPostPeriodic() {
    }

    /**
     * Helper to run a periodic body surrounded by pre/post hooks. Subclasses
     * may call this from {@link #periodic()}.
     * @param body the executable logic
     */
    protected final void runPeriodicWithHooks(Runnable body) {
        onPrePeriodic();
        try {
            if (body != null) {
                body.run();
        
            }} finally {
            onPostPeriodic();
        }
    }

    /**
     * Stop the subsystem. Default implementation calls {@link #onStop()}.
     */
    public void stop() {
        onStop();
    }

    /**
     * Hook: called when the subsystem is requested to stop. Default no-op.
     */
    protected void onStop() {
    }
}
