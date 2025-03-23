package ca.team4308.absolutelib.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Random;


public class Patterns {
    // Constants for pattern configuration
    public static final int DEFAULT_PATTERN_LENGTH = 30;
    public static final int DEFAULT_SCROLL_SPEED = 1;
    public static final double DEFAULT_BREATHE_PERIOD = 2.0;
    public static final double DEFAULT_BLINK_PERIOD = 0.2;

    /**
     * Creates a standard idle pattern.
     * Gentle breathing effect with the default idle color.
     * @return LEDPattern for idle state
     */
    public static LEDPattern idle() {
        return createBreathingPattern(LEDConstants.DEFAULT_IDLE_COLOR, LEDConstants.getBreathePeriod());
    }

    /**
     * Creates a warning pattern.
     * Fast blinking with warning color.
     * @return LEDPattern for warning state
     */
    public static LEDPattern warning() {
        return createBlinkingPattern(LEDConstants.DEFAULT_WARNING_COLOR, LEDConstants.getBlinkPeriod());
    }

    /**
     * Creates an error pattern.
     * Rapid blinking with error color.
     * @return LEDPattern for error state
     */
    public static LEDPattern error() {
        return createBlinkingPattern(LEDConstants.DEFAULT_ERROR_COLOR, LEDConstants.getBlinkPeriod() * 0.5);
    }

    /**
     * Creates a success pattern.
     * Solid success color with optional pulse.
     * @return LEDPattern for success state
     */
    public static LEDPattern success() {
        return createSolidPattern(LEDConstants.DEFAULT_SUCCESS_COLOR);
    }

    /**
     * Creates a rainbow chase pattern.
     * Scrolling rainbow effect with customizable speed.
     * @return LEDPattern with rainbow chase effect
     */
    public static LEDPattern rainbowChase() {
        return createRainbowPattern(
            LEDConstants.DEFAULT_RAINBOW_SATURATION,
            LEDConstants.DEFAULT_RAINBOW_VALUE,
            LEDConstants.DEFAULT_RAINBOW_SPEED,
            1.0 / LEDConstants.getPatternLength()
        );
    }

    /**
     * Creates an alliance-colored pattern (red or blue based on driver station)
     */
    public static LEDPattern getAlliancePattern() {
        Color color = getAllianceColor();
        return createSolidPattern(color);
    }

    /**
     * Creates a breathing alliance-colored pattern
     */
    public static LEDPattern getAllianceBreathing(double periodSeconds) {
        Color color = getAllianceColor();
        return createBreathingPattern(color, periodSeconds);
    }

    /**
     * Creates a rainbow pattern with customizable parameters
     */
    public static LEDPattern createRainbowPattern(int saturation, int value, double speedMPS, double spacing) {
        return new LEDPattern() {
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                int length = view.getLength();
                for (int i = 0; i < length; i++) {
                    int hue = (int)((i * 180.0 / length) % 180);
                    view.setHSV(i, hue, saturation, value);
                }
            }
        };
    }

    /**
     * Creates a breathing pattern that fades between colors
     */
    public static LEDPattern createBreathingPattern(Color baseColor, double periodSeconds) {
        return new LEDPattern() {
            private long startTime = System.currentTimeMillis();

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                double phase = (time % periodSeconds) / periodSeconds;
                double intensity = (Math.sin(phase * 2 * Math.PI) + 1) / 2;

                for (int i = 0; i < view.getLength(); i++) {
                    Color color = new Color(
                        baseColor.red * intensity,
                        baseColor.green * intensity,
                        baseColor.blue * intensity
                    );
                    view.setColor(i, color);
                }
            }
        };
    }

    /**
     * Creates a blinking pattern
     */
    public static LEDPattern createBlinkingPattern(Color color, double periodSeconds) {
        return new LEDPattern() {
            private long startTime = System.currentTimeMillis();

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                boolean isOn = ((time % periodSeconds) < (periodSeconds / 2));

                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, isOn ? color : new Color(0, 0, 0));
                }
            }
        };
    }

    /**
     * Creates a solid color pattern
     */
    public static LEDPattern createSolidPattern(Color color) {
        return new LEDPattern() {
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, color);
                }
            }
        };
    }

    /**
     * Creates a progress bar pattern
     */
    public static LEDPattern createProgressPattern(Color color, double progress) {
        return new LEDPattern() {
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                int activeLength = (int)(view.getLength() * progress);
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, i < activeLength ? color : new Color(0, 0, 0));
                }
            }
        };
    }

    /**
     * Creates a chasing dot pattern.
     * Single color dot moving along the strip.
     * @param color The color of the moving dot
     * @return LEDPattern with chasing dot effect
     */
    public static LEDPattern chasingDot(Color color) {
        return new LEDPattern() {
            private long startTime = System.currentTimeMillis();

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                int position = (int)(time * LEDConstants.DEFAULT_CHASE_SPEED * view.getLength()) % view.getLength();
                
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, i == position ? color : new Color(0, 0, 0));
                }
            }
        };
    }

    /**
     * Creates a scrolling idle pattern that transitions from base color to white.
     * @param baseColor The base color to fade from
     * @param scrollSpeed The speed of scrolling
     * @return LEDPattern for scrolling idle state
     */
    public static LEDPattern scrollingIdle(Color baseColor, int scrollSpeed) {
        return new LEDPattern() {
            private int offset = 0;

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                int patternLength = LEDConstants.getPatternLength();
                int trailLength = patternLength / 4; // Use 1/4 of pattern length for fade

                for (int i = 0; i < view.getLength(); i++) {
                    int position = (i + offset) % (patternLength * 3);
                    Utils.setIdlePatternColor(view, i, position, baseColor, patternLength, trailLength);
                }
                offset = (offset + scrollSpeed) % (patternLength * 3);
            }
        };
    }

    /**
     * Creates a default scrolling idle pattern using alliance colors.
     * @return LEDPattern for alliance-colored scrolling idle state
     */
    public static LEDPattern defaultScrollingIdle() {
        return scrollingIdle(getAllianceColor(), LEDConstants.getScrollSpeed());
    }

    /**
     * Creates the ALT-F4 pattern
     * This is really oringal guys
     * @param expandSpeed Speed of dot expansion (pixels per second)
     * @return LEDPattern for the ALT-F4 effect
     */
    public static LEDPattern altF4Pattern(double expandSpeed) {
        return new LEDPattern() {
            private final Random random = new Random();
            private final Color white = new Color(1, 1, 1);
            private final Color teamColor = getAllianceColor();
            private boolean isTeamColorPhase = true;
            private int[] expansionCenters = new int[0];
            private double[] expansionSizes = new double[0];
            private long lastUpdateTime = System.currentTimeMillis();
            private double timeSinceLastDot = 0;

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                long currentTime = System.currentTimeMillis();
                double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
                lastUpdateTime = currentTime;
                timeSinceLastDot += deltaTime;

                Color baseColor = isTeamColorPhase ? white : teamColor;
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, baseColor);
                }

                if (expansionCenters.length == 0 || isFullyExpanded(view.getLength())) {
                    if (timeSinceLastDot >= 1.0) { // Wait 1 second between phases
                        addNewExpansionPoint(view.getLength());
                        timeSinceLastDot = 0;
                    }
                }

                updateExpansions(deltaTime, expandSpeed, view);
            }

            private void addNewExpansionPoint(int length) {
                if (expansionCenters.length == 0 || isFullyExpanded(length)) {
                    isTeamColorPhase = !isTeamColorPhase; // Switch phases
                    expansionCenters = new int[]{random.nextInt(length)};
                    expansionSizes = new double[]{0};
                }
            }

            private void updateExpansions(double deltaTime, double speed, AddressableLEDBufferView view) {
                Color dotColor = isTeamColorPhase ? teamColor : white;
                
                // Update expansion sizes
                for (int i = 0; i < expansionSizes.length; i++) {
                    expansionSizes[i] += speed * deltaTime;
                    
                    // Apply expanded dots
                    int radius = (int)expansionSizes[i];
                    int center = expansionCenters[i];
                    
                    for (int j = center - radius; j <= center + radius; j++) {
                        int wrappedIndex = wrapIndex(j, view.getLength());
                        double distance = Math.abs(j - center);
                        double fade = 1.0 - (distance / radius);
                        
                        if (wrappedIndex >= 0 && wrappedIndex < view.getLength() && fade > 0) {
                            view.setColor(wrappedIndex, blendColors(
                                view.getLED(wrappedIndex),
                                dotColor,
                                fade
                            ));
                        }
                    }
                }
            }

            private boolean isFullyExpanded(int length) {
                if (expansionSizes.length == 0) return true;
                return expansionSizes[0] >= length / 2.0;
            }

            private int wrapIndex(int index, int length) {
                if (index < 0) return length + (index % length);
                return index % length;
            }

            private Color blendColors(Color c1, Color c2, double blend) {
                return new Color(
                    c1.red + (c2.red - c1.red) * blend,
                    c1.green + (c2.green - c1.green) * blend,
                    c1.blue + (c2.blue - c1.blue) * blend
                );
            }
        };
    }

    /**
     * Creates the default ALT-F4 pattern with standard speed.
     * @return LEDPattern for the ALT-F4 effect
     */
    public static LEDPattern altF4() {
        return altF4Pattern(10.0); 
    }

    // Helper methods
    private static Color getAllianceColor() {
        Alliance alliance = DriverStation.getAlliance();
        if (alliance == Alliance.Blue) {
            return new Color(0, 0, 1); // Blue
        }
        return new Color(1, 0, 0); // Red
    }
}
