package ca.team4308.absolutelib.math.trajectories.impl;

public class InterpolatingDoubleTreeMap {
	private final java.util.TreeMap<Double, Double> map = new java.util.TreeMap<>();

	public InterpolatingDoubleTreeMap() {
	}

	/**
	 * Puts a key/value pair into the map.
	 */
	public void put(double key, double value) {
		map.put(key, value);
	}

	/**
	 * Puts a boxed key/value pair into the map.
	 */
	public void put(Double key, Double value) {
		if (key == null || value == null) {
			throw new IllegalArgumentException("Key and value must not be null");
		}
		map.put(key, value);
	}

	/**
	 * Returns an interpolated value for the given key.
	 * If key is outside range, it extrapolates from the nearest interval.
	 */
	public Double get(double key) {
		if (map.isEmpty()) {
			return null;
		}

		if (map.containsKey(key)) {
			return map.get(key);
		}

		java.util.Map.Entry<Double, Double> lower = map.floorEntry(key);
		java.util.Map.Entry<Double, Double> upper = map.ceilingEntry(key);

		if (lower == null && upper == null) {
			return null;
		}
		if (lower == null) {
			return upper.getValue();
		}
		if (upper == null) {
			return lower.getValue();
		}

		if (lower.getKey().equals(upper.getKey())) {
			return lower.getValue();
		}

		double x0 = lower.getKey();
		double y0 = lower.getValue();
		double x1 = upper.getKey();
		double y1 = upper.getValue();
		double ratio = (key - x0) / (x1 - x0);
		return y0 + ratio * (y1 - y0);
	}

	/**
	 * Returns the value at exact key or null.
	 */
	public Double get(Double key) {
		return key == null ? null : get(key.doubleValue());
	}

	public int size() {
		return map.size();
	}

	public boolean isEmpty() {
		return map.isEmpty();
	}

	public void clear() {
		map.clear();
	}

	public Double firstKey() {
		if (map.isEmpty()) {
			return null;
		}
		return map.firstKey();
	}

	public Double lastKey() {
		if (map.isEmpty()) {
			return null;
		}
		return map.lastKey();
	}
	}
