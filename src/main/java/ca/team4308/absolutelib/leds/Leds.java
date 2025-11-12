package ca.team4308.absolutelib.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class Leds {
    private final int port;
    private final int length;
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private boolean isStarted = false;

    /**
     * Creates a new LED strip controller.
     * @param port The PWM port the LED strip is connected to
     * @param length The number of LEDs in the strip
     */

    public Leds(int port, int length) {
        this.port = port;
        this.length = length;
        this.ledStrip = new AddressableLED(port);
        this.ledBuffer = new AddressableLEDBuffer(length);
        
        // Initialize the LED strip
        this.ledStrip.setLength(length);
        this.ledStrip.setData(ledBuffer);
    }

    /**
     * Starts the LED output.
     * Must be called after configuration and before setting colors.
     */

    public void start() {
        if (!isStarted) {
            ledStrip.start();
            isStarted = true;
        }
    }

    /**
     * Stops the LED output.
     */

    public void stop() {
        if (isStarted) {
            ledStrip.stop();
            isStarted = false;
        }
    }


    public void setColor(int index, Color color)  {
        if (index >= 0 && index < length) {
            ledBuffer.setLED(index, color);
        }
    }

    /**
     * Sets the color of a specific LED.
     * @param index The LED index
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */



    public void setRGB(int index, int r, int g, int b) {
        if (index >= 0 && index < length) {
            ledBuffer.setRGB(index, r, g, b);
        }
    }

    /**
     * Sets the HSV color of a specific LED.
     * @param index The LED index
     * @param h Hue (0-180)
     * @param s Saturation (0-255)
     * @param v Value (0-255) "Brightness" or "Value"
     */

    public void setHSV(int index, int h, int s, int v) {
        if (index >= 0 && index < length) {
            ledBuffer.setHSV(index, h, s, v);
        }
    }

    /**
     * Updates the LED strip with the current buffer contents.
     */


    public void update() {
        if (isStarted) {
            ledStrip.setData(ledBuffer);
        }
    }

    
    /**
     * Gets the length of the LED strip.
     * @return The number of LEDs in the strip
     */

    public int getLength() {
        return length;
    }

    /**
     * Gets the port number the LED strip is connected to.
     * @return The PWM port number
     */
    public int getPort() {
        return port;
    }

    /**
     * Clears all LEDs by setting them to off (0,0,0).
     */

    public void clear() {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        update();
    }

    /**
     * Creates a view into a section of the LED strip.
     * @param startIndex The starting LED index
     * @param length The number of LEDs in the view
     * @return A new AddressableLEDBufferView
     */
    public AddressableLEDBufferView createBufferView(int startIndex, int length) {
        return new AddressableLEDBufferView(ledBuffer, startIndex, length);
    }

    /**
     * Creates multiple equal-sized views of the LED strip.
     * @param numSections The number of sections to create
     * @return Array of AddressableLEDBufferView objects
     */
    public AddressableLEDBufferView[] splitIntoSections(int numSections) {
        if (numSections <= 0 || length % numSections != 0) {
            throw new IllegalArgumentException("Invalid number of sections");
        }

        int sectionLength = length / numSections;
        AddressableLEDBufferView[] views = new AddressableLEDBufferView[numSections];
        
        for (int i = 0; i < numSections; i++) {
            views[i] = createBufferView(i * sectionLength, sectionLength);
        }
        
        return views;
    }

    /**
     * Applies a solid color to a specific buffer view.
     * 
     * 
     * @param view The buffer view to apply the color to
     * @param pattern The pattern to apply to the view
     * 
     * 
     */

    public void applyToView(AddressableLEDBufferView view, LEDPattern pattern) {
        pattern.applyTo(view);
    }
    
        /**
     * Applies a solid color to a specific buffer view.
     * 
     * 
     * @param view The buffer view to apply the color to
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */

    public void applyToView(AddressableLEDBufferView view, int r, int g, int b) {
        for (int i = 0; i < view.getLength(); i++) {
            view.setRGB(i, r, g, b);
        }
    }

    /**
     * Applies an HSV color to a specific buffer view.
     * @param view The buffer view to apply the color to
     * @param h Hue (0-180)
     * @param s Saturation (0-255)
     * @param v Value/Brightness (0-255)
     */
    public void applyHSVToView(AddressableLEDBufferView view, int h, int s, int v) {
        for (int i = 0; i < view.getLength(); i++) {
            view.setHSV(i, h, s, v);
        }
    }

    /**
     * Applies a custom operation to each LED in a buffer view.
     * @param view The buffer view to apply the operation to
     * @param operation The operation to apply to each LED index
     */
    public void applyOperationToView(AddressableLEDBufferView view, LEDOperation operation) {
        for (int i = 0; i < view.getLength(); i++) {
            operation.apply(view, i);
        }
    }

    /**
     * Functional interface for custom LED operations
     */
    @FunctionalInterface
    public interface LEDOperation {
        void apply(AddressableLEDBufferView view, int index);
    }

    /**
     * Applies a pattern to a specific buffer view.
     * @param view The buffer view to apply the pattern to
     * @param pattern The pattern to apply to the view
     */
    public void applyPattern(AddressableLEDBufferView view, LEDPattern pattern) {
        pattern.applyTo(view);
    }
}
