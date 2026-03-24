package ca.team4308.absolutelib.math.trajectories.impl;

public class Pose3d {
	private final ca.team4308.absolutelib.math.Vector3 translation;
	private final Rotation3d rotation;

	public Pose3d() {
		this(new ca.team4308.absolutelib.math.Vector3(0.0, 0.0, 0.0), new Rotation3d());
	}

	public Pose3d(double x, double y, double z, Rotation3d rotation) {
		this(new ca.team4308.absolutelib.math.Vector3(x, y, z), rotation);
	}

	public Pose3d(ca.team4308.absolutelib.math.Vector3 translation, Rotation3d rotation) {
		if (translation == null) {
			throw new IllegalArgumentException("Translation cannot be null");
		}
		if (rotation == null) {
			throw new IllegalArgumentException("Rotation cannot be null");
		}
		this.translation = translation;
		this.rotation = rotation;
	}

	public ca.team4308.absolutelib.math.Vector3 getTranslation() {
		return translation;
	}

	public Rotation3d getRotation() {
		return rotation;
	}

	public Pose3d relativeTo(Pose3d other) {
		if (other == null) {
			throw new IllegalArgumentException("Other pose cannot be null");
		}
		ca.team4308.absolutelib.math.Vector3 relTranslation = translation.sub(other.translation);
		Rotation3d relRotation = rotation.relativeTo(other.rotation);
		return new Pose3d(relTranslation, relRotation);
	}

	public Pose3d transformBy(Pose3d other) {
		if (other == null) {
			throw new IllegalArgumentException("Other pose cannot be null");
		}
		ca.team4308.absolutelib.math.Vector3 newTranslation = translation.add(other.translation);
		Rotation3d newRotation = rotation.rotateBy(other.rotation);
		return new Pose3d(newTranslation, newRotation);
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj) {
			return true;
		}
		if (!(obj instanceof Pose3d)) {
			return false;
		}
		Pose3d o = (Pose3d) obj;
		return translation.equals(o.translation) && rotation.equals(o.rotation);
	}

	@Override
	public int hashCode() {
		int result = translation != null ? translation.hashCode() : 0;
		result = 31 * result + (rotation != null ? rotation.hashCode() : 0);
		return result;
	}

	@Override
	public String toString() {
		return String.format("Pose3d{translation=%s, rotation=%s}", translation, rotation);
	}
    
}
