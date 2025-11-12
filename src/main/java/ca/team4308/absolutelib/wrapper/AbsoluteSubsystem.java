package ca.team4308.absolutelib.wrapper;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Extends SubsystemBase to provide logging capabilities and more helper methods.
 */
public abstract class AbsoluteSubsystem extends SubsystemBase {

    /** Provide a Sendable for telemetry/logging; may return null if unused. */
    public abstract Sendable log();

    private volatile LoggingBackend backend;
    private final Map<String, Long> lastLogMs = new HashMap<>();
    private final Set<String> loggedOnce = new HashSet<>();

    public AbsoluteSubsystem() {
        this.backend = new DriverStationBackend();
    }

    /*
     * Returns the base log channel for this subsystem.
     */
    protected String getLogChannelBase() {
        String n = getName();
        if (n == null || n.isEmpty()) n = getClass().getSimpleName();
        return "/subsystems/" + n;
        }

        /** Add a value to SmartDashboard under this subsystem's namespace. any type of data is supported. Please use this.*/
        protected void SDAdd(String name, Object value) {
        String fullKey = getLogChannelBase() + "/" + name;
        if (value instanceof Sendable) {
            SmartDashboard.putData(fullKey, (Sendable) value);
        } else if (value instanceof Double) {
            SmartDashboard.putNumber(fullKey, (Double) value);
        } else if (value instanceof Integer) {
            SmartDashboard.putNumber(fullKey, ((Integer) value).doubleValue());
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(fullKey, (Boolean) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(fullKey, (String) value);
        } else {
            throw new IllegalArgumentException("Unsupported data type for SmartDashboard: " + value.getClass().getName());
        }
        }

    /** Log info.
     * 
     */
    protected void logInfo(String message) {
        backend.info(getLogChannelBase(), message);
    }

    /** Log a warning. 
     * 
     */
    protected void logWarn(String message) {
        backend.warn(getLogChannelBase(), message);
    }

    /** Log an error. 
     * 
    */
    protected void logError(String message) {
        backend.error(getLogChannelBase(), message, null);
    }

    /** Log an error with throwable */
    protected void logError(String message, Throwable t) {
        backend.error(getLogChannelBase(), message, t);
    }

    protected void logNumber(String key, double value) {
        backend.info(getLogChannelBase() + "/" + key, Double.toString(value));
    }
    protected void logBoolean(String key, boolean value) {
        backend.info(getLogChannelBase() + "/" + key, Boolean.toString(value));
    }

    /** Log a message only the first time for the given key. */
    protected void logOnce(String key, String message) {
        if (loggedOnce.add(key)) {
            logInfo("[once] " + message);
        }
    }

    /** Throttled log: only logs if at least minIntervalMs since last time for the key. */
    protected void logThrottle(String key, long minIntervalMs, String message) {
        long now = nowMs();
        Long prev = lastLogMs.get(key);
        if (prev == null || now - prev >= minIntervalMs) {
            lastLogMs.put(key, now);
            logInfo("[throttled] " + message);
        }
    }

    /** Wrap a runnable and log any thrown exception with a label. */
    protected void runSafely(String label, Runnable body) {
        try { body.run(); }
        catch (Throwable t) { logError("Exception in " + label, t); }
    }

    private static long nowMs() { return (long)(Timer.getFPGATimestamp() * 1_000.0); }
    @SuppressWarnings("unused")
    private String formatPrefix() { return "[" + getLogChannelBase() + "] "; }
    private static String stackTrace(Throwable t) {
        try (StringWriter sw = new StringWriter(); PrintWriter pw = new PrintWriter(sw)) {
            t.printStackTrace(pw);
            return sw.toString();
        } catch (Exception e) {
            return t.toString();
        }
    }

    protected interface LoggingBackend {
        void info(String channel, String message);
        void warn(String channel, String message);
        void error(String channel, String message, Throwable t);
    }

    /** Fallback backend that reports via DriverStation only. */
    private static class DriverStationBackend implements LoggingBackend {
        @Override public void info(String channel, String message) { 
            Logger.recordOutput(channel + "/info", message);
        }
        @Override public void warn(String channel, String message) {
            Logger.recordOutput(channel + "/warn", message);
        }
        @Override public void error(String channel, String message, Throwable t) {
            Logger.recordOutput(channel + "/error", message + (t != null ? ("\n" + stackTrace(t)) : ""));
            DriverStation.reportError(channel + " Has an error " + message + "Check /error for full error.", true);
        } 
    }

    /** Allows external code to supply a custom backend implementation. */
    public void setCustomLoggingBackend(LoggingBackend custom) { if (custom != null) this.backend = custom; }

    public void initialize() { onInitialize(); }

    protected void onInitialize() {}

    // Hoesntly unused but whatever have them for ltr

    protected void onPrePeriodic() {}
    protected void onPostPeriodic() {}

    /**
     * Helper to run a periodic body surrounded by pre/post hooks.
     * Subclasses may call this from {@link #periodic()}.
     */

    protected final void runPeriodicWithHooks(Runnable body) {
        onPrePeriodic();
        try { if (body != null) body.run(); }
        finally { onPostPeriodic(); }
    }

    /** Stop the subsystem. Default implementation calls {@link #onStop()}. */
    public void stop() { onStop(); }

    /** Hook: called when the subsystem is requested to stop. Default no-op. */
    protected void onStop() {}
}
