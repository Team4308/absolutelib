package ca.team4308.absolutelib.leds;

import edu.wpi.first.wpilibj.util.Color;


public class Utils  {



    /**
     * Calculates a fade value based on position in pattern.
     * @param position Current position in pattern
     * @param patternLength Total length of pattern
     * @param trailLength Length of fade trail
     * @return Fade value between 0.0 and 1.0
     */
    public static double getFadeValue(int position, int patternLength, int trailLength) {
        if (trailLength <= 0 || patternLength <= 0) {
            return 0.0;
        }

        if (position < trailLength) {
            return (double) position / trailLength;
        } else if (position >= patternLength - trailLength) {
            return (double) (patternLength - position) / trailLength;
        }
        return 1.0;
    }

    /**
     * Creates a breathing pattern that fades between a base color and white.
     * @param buffer LED buffer view to modify
     * @param position Current animation position
     * @param baseColor Base color to fade from
     * @param patternLength Length of the complete pattern
     * @param trailLength Length of the fade trail
     */
    public static void createBreathingPattern(AddressableLEDBufferView buffer, int position, 
            Color baseColor, int patternLength, int trailLength) {
        if (buffer == null || baseColor == null) {
            return;
        }

        for (int i = 0; i < buffer.getLength(); i++) {
            int adjustedPosition = position % (patternLength * 3);
            
            if (adjustedPosition < patternLength) {
                // First phase: fade up
                double fade = getFadeValue(adjustedPosition, patternLength, trailLength);
                double blendFactor = fade * 0.3;
                setBlendedColor(buffer, i, baseColor, blendFactor);
            } else if (adjustedPosition < patternLength * 2) {
                // Second phase: middle intensity
                double fade = getFadeValue(adjustedPosition - patternLength, patternLength, trailLength);
                double blendFactor = 0.3 + (fade * 0.4);
                setBlendedColor(buffer, i, baseColor, blendFactor);
            } else {
                // Third phase: fade to maximum
                double fade = getFadeValue(adjustedPosition - (patternLength * 2), patternLength, trailLength);
                double blendFactor = 0.7 + (fade * 0.3);
                setBlendedColor(buffer, i, baseColor, blendFactor);
            }
        }
    }

    /**
     * Sets the color for a scrolling idle pattern that fades between a base color and white.
     * @param buffer LED buffer to modify
     * @param index LED index
     * @param position Current pattern position
     * @param baseColor Base color to fade from
     * @param patternLength Total length of pattern
     * @param trailLength Length of fade trail
     */
    public static void setIdlePatternColor(AddressableLEDBufferView buffer, int index, 
            int position, Color baseColor, int patternLength, int trailLength) {
        if (position < patternLength) {
            double fade = getFadeValue(position, patternLength, trailLength);
            double blendFactor = fade * 0.3;
            setBlendedColor(buffer, index, baseColor, blendFactor);
        } else if (position < patternLength * 2) {
            double fade = getFadeValue(position - patternLength, patternLength, trailLength);
            double blendFactor = 0.3 + (fade * 0.4);
            setBlendedColor(buffer, index, baseColor, blendFactor);
        } else {
            double fade = getFadeValue(position - (patternLength * 2), patternLength, trailLength);
            double blendFactor = 0.7 + (fade * 0.3);
            setBlendedColor(buffer, index, baseColor, blendFactor);
        }
    }

    /**
     * Sets a color blended between the base color and white.
     * @param buffer LED buffer to modify
     * @param index LED index
     * @param baseColor Base color to blend from
     * @param blendFactor Blend factor (0.0 = base color, 1.0 = white)
     */
    private static void setBlendedColor(AddressableLEDBufferView buffer, int index, Color baseColor, double blendFactor) {
        blendFactor = clamp(blendFactor, 0.0, 1.0);
        
        Color color = new Color(
            baseColor.red + (blendFactor * (1.0 - baseColor.red)),
            baseColor.green + (blendFactor * (1.0 - baseColor.green)),
            baseColor.blue + (blendFactor * (1.0 - baseColor.blue))
        );
        
        buffer.setColor(index, color); // Assuming setColor is the correct method
    }

    /**
     * Clamps a value between a minimum and maximum.
     * @param value Value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped value
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
