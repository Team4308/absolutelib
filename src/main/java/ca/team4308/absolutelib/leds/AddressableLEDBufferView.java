package ca.team4308.absolutelib.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Represents a view into a portion of an AddressableLEDBuffer.
 * Allows treating a section of LEDs as its own buffer.
 */
public class AddressableLEDBufferView {
    private final AddressableLEDBuffer buffer;
    private final int startIndex;
    private final int length;

    /**
     * Creates a new view into an LED buffer.
     * @param buffer The parent LED buffer
     * @param startIndex The starting index in the parent buffer
     * @param length The number of LEDs in this view
     */
    public AddressableLEDBufferView(AddressableLEDBuffer buffer, int startIndex, int length) {
        if (startIndex < 0 || startIndex + length > buffer.getLength()) {
            throw new IllegalArgumentException("Invalid buffer view range");
        }
        this.buffer = buffer;
        this.startIndex = startIndex;
        this.length = length;
    }

    /**
     * Sets the RGB values for an LED in this view.
     * @param index The index within this view
     * @param color Red value (0-255)
     */

    public void setColor(int index, Color color) {
        if (isValidIndex(index)) {
            buffer.setLED(index, color);
        }
    }


    /**
     * Sets the RGB values for an LED in this view.
     * @param index The index within this view
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */
    public void setRGB(int index, int r, int g, int b) {
        if (isValidIndex(index)) {
            buffer.setRGB(startIndex + index, r, g, b);
        }
    }

    /**
     * Sets the HSV values for an LED in this view.
     * @param index The index within this view
     * @param h Hue (0-180)
     * @param s Saturation (0-255)
     * @param v Value/Brightness (0-255)
     */
    public void setHSV(int index, int h, int s, int v) {
        if (isValidIndex(index)) {
            buffer.setHSV(startIndex + index, h, s, v);
        }
    }

    /**
     * Gets the color of an LED in this view.
     * @param index The index within this view
     * @return The Color object representing the LED's color
     */
    public Color getLED(int index) {
        if (isValidIndex(index)) {
            return buffer.getLED(startIndex + index);
        }
        return new Color(0, 0, 0);
    }

    /**
     * Gets the length of this buffer view.
     * @return The number of LEDs in this view
     */
    public int getLength() {
        return length;
    }

    /**
     * Gets the starting index of this view in the parent buffer.
     * @return The start index
     */
    public int getStartIndex() {
        return startIndex;
    }

    private boolean isValidIndex(int index) {
        return index >= 0 && index < length;
    }
}
