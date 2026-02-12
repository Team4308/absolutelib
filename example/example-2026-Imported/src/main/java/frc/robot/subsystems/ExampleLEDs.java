package frc.robot.subsystems;

import ca.team4308.absolutelib.leds.*;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Example LED subsystem demonstrating addressable LED patterns. Uses the
 * Patterns factory class for various LED effects.
 */
public class ExampleLEDs extends AbsoluteSubsystem {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView view;

    private LEDPattern currentPattern;
    private String currentPatternName = "Idle";

    // LED configuration
    private static final int LED_PORT = 0;
    private static final int LED_LENGTH = 60;

    public ExampleLEDs() {
        super();

        // Initialize LED hardware
        led = new AddressableLED(LED_PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(LED_LENGTH);
        led.setData(buffer);
        led.start();

        // Create view for pattern application
        view = new AddressableLEDBufferView(buffer, 0, LED_LENGTH);

        // Start with idle pattern
        currentPattern = Patterns.idle();
    }

    @Override
    public void periodic() {
        // Apply current pattern and update LEDs
        if (currentPattern != null) {
            currentPattern.applyTo(view);
        }
        led.setData(buffer);
    }

    /**
     * Set idle pattern - gentle breathing effect.
     */
    public void setIdle() {
        currentPattern = Patterns.idle();
        currentPatternName = "Idle";
    }

    /**
     * Set error pattern - rapid red blinking.
     */
    public void setError() {
        currentPattern = Patterns.error();
        currentPatternName = "Error";
    }

    /**
     * Set success pattern - solid green.
     */
    public void setSuccess() {
        currentPattern = Patterns.success();
        currentPatternName = "Success";
    }

    /**
     * Set warning pattern - fast yellow blinking.
     */
    public void setWarning() {
        currentPattern = Patterns.warning();
        currentPatternName = "Warning";
    }

    /**
     * Set rainbow chase pattern.
     */
    public void setRainbow() {
        currentPattern = Patterns.rainbowChase();
        currentPatternName = "Rainbow";
    }

    /**
     * Set alliance-based pattern (red or blue based on driver station).
     */
    public void setAlliance() {
        currentPattern = Patterns.getAlliancePattern();
        currentPatternName = "Alliance";
    }

    /**
     * Set alliance-based breathing pattern.
     */
    public void setAllianceBreathing() {
        currentPattern = Patterns.getAllianceBreathing(2.0);
        currentPatternName = "Alliance Breathing";
    }

    /**
     * Set a solid color.
     */
    public void setSolidColor(Color color) {
        currentPattern = Patterns.createSolidPattern(color);
        currentPatternName = "Solid";
    }

    /**
     * Set a progress bar pattern (0.0 to 1.0).
     */
    public void setProgress(double progress, Color color) {
        currentPattern = Patterns.createProgressPattern(color, progress);
        currentPatternName = "Progress";
    }

    /**
     * Set chasing dot pattern.
     */
    public void setChasingDot(Color color) {
        currentPattern = Patterns.chasingDot(color);
        currentPatternName = "Chasing Dot";
    }

    public String getCurrentPatternName() {
        return currentPatternName;
    }

    @Override
    public Sendable log() {
        return builder -> {
            builder.setSmartDashboardType("ExampleLEDs");
            builder.addStringProperty("CurrentPattern", () -> currentPatternName, null);
            builder.addIntegerProperty("LEDCount", () -> LED_LENGTH, null);
        };
    }
}
