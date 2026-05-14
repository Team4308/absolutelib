package ca.team4308.coprocessor;

public class Config {
    
    // Allows overriding via environment variables or falling back to defaults
    public static final int TEAM_NUMBER = getIntEnv("TEAM_NUMBER", 4308);
    // Network settings
    public static final int TCP_PORT = getIntEnv("TCP_PORT", 5801);
    public static final int HTTP_PORT = getIntEnv("HTTP_PORT", 5805);
    public static final boolean IS_SIMULATION = getBooleanEnv("IS_SIMULATION", false);
    public static final boolean BINARY_PROTOCOL_ENABLED = getBooleanEnv("BINARY_PROTOCOL_ENABLED", true);

    // Feature toggles
    public static final boolean PREDICTION_ENABLED = getBooleanEnv("PREDICTION_ENABLED", true);
    public static final int PREDICTION_ITERATIONS = getIntEnv("PREDICTION_ITERATIONS", 1);
    
    // Smoothing (0.0 to 1.0, lower is smoother but more lag. 1.0 = no smoothing)
    public static final double SMOOTHING_EMA_ALPHA = getDoubleEnv("SMOOTHING_EMA_ALPHA", 0.3);
    public static final double SMOOTHING_RESET_THRESHOLD_DEG = getDoubleEnv("SMOOTHING_RESET_THRESHOLD_DEG", 5.0);

    // Logging
    public static final boolean LOG_TO_FILE = getBooleanEnv("LOG_TO_FILE", true);
    public static final String LOG_FILE_PATH = getStringEnv("LOG_FILE_PATH", "trajectory_log.jsonl");

    // Determines the NT4 Server address depending on the mode
    public static String getRobotAddress() {
        if (IS_SIMULATION) {
            return "127.0.0.1";
        } else {
            // e.g. 10.43.8.2
            int te = TEAM_NUMBER / 100;
            int am = TEAM_NUMBER % 100;
            return "10." + te + "." + am + ".2";
        }
    }

    private static int getIntEnv(String key, int def) {
        String val = System.getenv(key);
        if (val != null && !val.trim().isEmpty()) {
            try {
                return Integer.parseInt(val);
            } catch (NumberFormatException ignored) {}
        }
        return def;
    }

    private static double getDoubleEnv(String key, double def) {
        String val = System.getenv(key);
        if (val != null && !val.trim().isEmpty()) {
            try {
                return Double.parseDouble(val);
            } catch (NumberFormatException ignored) {}
        }
        return def;
    }

    private static String getStringEnv(String key, String def) {
        String val = System.getenv(key);
        if (val != null && !val.trim().isEmpty()) {
            return val;
        }
        return def;
    }

    private static boolean getBooleanEnv(String key, boolean def) {
        String val = System.getenv(key);
        if (val != null && !val.trim().isEmpty()) {
            return Boolean.parseBoolean(val);
        }
        return def;
    }
}
