package ca.team4308.absolutelib.leds;

import edu.wpi.first.wpilibj.util.Color;


public final class LEDConstants {
    // Hardware Configuration
    private static int ELEVATOR_LENGTH = 30;
    private static int FUNNEL_VERT_LENGTH = 15;
    private static int FUNNEL_HORI_LENGTH = 15;
    
    // Pattern Configuration
    private static int PATTERN_LENGTH = 30;
    private static int SCROLL_SPEED = 1;
    private static double BREATHE_PERIOD = 2.0;
    private static double BLINK_PERIOD = 0.2;
    
    // Brightness Levels
    private static double MAX_BRIGHTNESS = 1.0;
    private static double MEDIUM_BRIGHTNESS = 0.5;
    private static double LOW_BRIGHTNESS = 0.3;

    // Default Pattern Colors
    public static final Color DEFAULT_IDLE_COLOR = new Color(0.5, 0.5, 0.5);
    public static final Color DEFAULT_WARNING_COLOR = new Color(1.0, 0.5, 0.0);
    public static final Color DEFAULT_ERROR_COLOR = new Color(1.0, 0.0, 0.0);
    public static final Color DEFAULT_SUCCESS_COLOR = new Color(0.0, 1.0, 0.0);

    // Default Pattern SettingsA
    public static final int DEFAULT_RAINBOW_SATURATION = 255;
    public static final int DEFAULT_RAINBOW_VALUE = 128;
    public static final double DEFAULT_RAINBOW_SPEED = 0.5;
    public static final double DEFAULT_CHASE_SPEED = 1.0;

    // Getters
    public static int getElevatorLength() { return ELEVATOR_LENGTH; }
    public static int getFunnelVertLength() { return FUNNEL_VERT_LENGTH; }
    public static int getFunnelHoriLength() { return FUNNEL_HORI_LENGTH; }
    public static int getPatternLength() { return PATTERN_LENGTH; }
    public static int getScrollSpeed() { return SCROLL_SPEED; }
    public static double getBreathePeriod() { return BREATHE_PERIOD; }
    public static double getBlinkPeriod() { return BLINK_PERIOD; }
    public static double getMaxBrightness() { return MAX_BRIGHTNESS; }
    public static double getMediumBrightness() { return MEDIUM_BRIGHTNESS; }
    public static double getLowBrightness() { return LOW_BRIGHTNESS; }

    // Setters with validation
    public static void setElevatorLength(int length) {
        ELEVATOR_LENGTH = Math.max(1, length);
    }
    
    public static void setFunnelVertLength(int length) {
        FUNNEL_VERT_LENGTH = Math.max(1, length);
    }
    
    public static void setFunnelHoriLength(int length) {
        FUNNEL_HORI_LENGTH = Math.max(1, length);
    }
    
    public static void setPatternLength(int length) {
        PATTERN_LENGTH = Math.max(1, length);
    }
    
    public static void setScrollSpeed(int speed) {
        SCROLL_SPEED = Math.max(0, speed);
    }
    
    public static void setBreathePeriod(double period) {
        BREATHE_PERIOD = Math.max(0.1, period);
    }
    
    public static void setBlinkPeriod(double period) {
        BLINK_PERIOD = Math.max(0.1, period);
    }
    
    public static void setBrightness(double brightness) {
        MAX_BRIGHTNESS = clamp(brightness, 0.0, 1.0);
        MEDIUM_BRIGHTNESS = MAX_BRIGHTNESS * 0.5;
        LOW_BRIGHTNESS = MAX_BRIGHTNESS * 0.3;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private LEDConstants() {
        // Prevent instantiation
    }
}
