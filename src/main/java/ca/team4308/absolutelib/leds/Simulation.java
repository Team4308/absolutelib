package ca.team4308.absolutelib.leds;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class Simulation {
    public SimDevice m_simDevice;
    public SimDouble m_simR;
    public SimDouble m_simG;
    public SimDouble m_simB;
    public SimDouble m_simBrightness;

    /**
     * Sets up the LED simulation device and its channels.
     * Creates simulation variables for RGB values and brightness.
     */
    public void setUp() {
        m_simDevice = SimDevice.create("LEDs");
        if (m_simDevice == null) {
            System.out.println("Could not create LED simulation device");
            return;
        }
        m_simR = m_simDevice.createDouble("R", SimDevice.Direction.kOutput, 0.0);
        m_simG = m_simDevice.createDouble("G", SimDevice.Direction.kOutput, 0.0);
        m_simB = m_simDevice.createDouble("B", SimDevice.Direction.kOutput, 0.0);
        m_simBrightness = m_simDevice.createDouble("Brightness", SimDevice.Direction.kOutput, 0.0);
    }

    /**
     * Updates the simulation with current LED buffer values.
     * Calculates average RGB values and brightness across all LEDs.
     *
     * @param m_buffer The LED buffer containing current LED states
     * @param ledStrip The LED strip object being simulated
     */
    public void update(AddressableLEDBuffer m_buffer, AddressableLED ledStrip) {
        double totalR = 0, totalG = 0, totalB = 0;

        for (int i = 0; i < m_buffer.getLength(); i++) {
            Color color = m_buffer.getLED(i);
            totalR += color.red;
            totalG += color.green;
            totalB += color.blue;
        }

        double length = m_buffer.getLength();
        m_simR.set(totalR / length);
        m_simG.set(totalG / length);
        m_simB.set(totalB / length);
        m_simBrightness.set(Math.max(Math.max(totalR, totalG), totalB) / length);
    }

}
