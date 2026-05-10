package ca.team4308.coprocessor;

import java.util.concurrent.atomic.AtomicInteger;

public class LatencyFilter {
    private final double[] samples;
    private final AtomicInteger index = new AtomicInteger(0);
    private final int windowSize;
    private volatile double lastFilteredValue = 0.0;

    public LatencyFilter(int windowSize) {
        this.windowSize = windowSize;
        this.samples = new double[windowSize];
    }

    public double addSample(double value) {
        int idx = index.getAndUpdate(i -> (i + 1) % windowSize);
        samples[idx] = value;
        
        double sum = 0.0;
        for (double sample : samples) {
            sum += sample;
        }
        lastFilteredValue = sum / windowSize;
        return lastFilteredValue;
    }

    public double getLastFilteredValue() {
        return lastFilteredValue;
    }

    public void reset() {
        index.set(0);
        for (int i = 0; i < samples.length; i++) {
            samples[i] = 0.0;
        }
        lastFilteredValue = 0.0;
    }
}
