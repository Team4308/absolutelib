package ca.team4308.absolutelib.math.trajectories.impl;

public class Rotation3d {
	private final double x;
	private final double y;
	private final double z;

	public Rotation3d() {
		this(0.0, 0.0, 0.0);
	}

	public Rotation3d(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getZ() {
		return z;
	}

	public Rotation3d div(double scalar) {
		if (scalar == 0.0) {
			throw new IllegalArgumentException("Division by zero");
		}
		return new Rotation3d(x / scalar, y / scalar, z / scalar);
	}

	public Rotation3d times(double scalar) {
		return fromAxisAngle(getAxis(), getAngle() * scalar);
	}

	public Rotation3d plus(Rotation3d other) {
		return rotateBy(other);
	}

	public Rotation3d minus(Rotation3d other) {
		return rotateBy(other.unaryMinus());
	}

	public Rotation3d unaryMinus() {
		return fromQuaternion(toQuaternion().inverse());
	}

	public Rotation3d rotateBy(Rotation3d other) {
		Quaternion result = toQuaternion().multiply(other.toQuaternion());
		return fromQuaternion(result);
	}

	public Rotation3d relativeTo(Rotation3d other) {
		// Rotation that brings other to this.
		return this.rotateBy(other.unaryMinus());
	}

	public Rotation3d interpolate(Rotation3d endValue, double t) {
		if (t <= 0.0) {
			return this;
		}
		if (t >= 1.0) {
			return endValue;
		}

		Quaternion q1 = toQuaternion();
		Quaternion q2 = endValue.toQuaternion();
		Quaternion interp = q1.slerp(q2, t);
		return fromQuaternion(interp);
	}

	public double getAngle() {
		Quaternion q = toQuaternion();
		double angle = 2.0 * Math.acos(clamp(q.w, -1.0, 1.0));
		if (angle > Math.PI) {
			angle = 2.0 * Math.PI - angle;
		}
		return angle;
	}

	public ca.team4308.absolutelib.math.Vector3 getAxis() {
		Quaternion q = toQuaternion();
		double angle = getAngle();
		double sinHalfAngle = Math.sin(angle / 2.0);
		if (Math.abs(sinHalfAngle) < 1e-12 || angle == 0.0) {
			return new ca.team4308.absolutelib.math.Vector3(1.0, 0.0, 0.0);
		}
		return new ca.team4308.absolutelib.math.Vector3(q.x / sinHalfAngle, q.y / sinHalfAngle, q.z / sinHalfAngle).normalize();
	}

	public Angle getMeasureAngle() {
		return new Angle(getAngle());
	}

	public Angle getMeasureX() {
		return new Angle(getX());
	}

	public Angle getMeasureY() {
		return new Angle(getY());
	}

	public Angle getMeasureZ() {
		return new Angle(getZ());
	}

	public Quaternion getQuaternion() {
		return toQuaternion();
	}

	public Matrix3d toMatrix() {
		Quaternion q = toQuaternion().normalize();
		double xx = q.x * q.x;
		double yy = q.y * q.y;
		double zz = q.z * q.z;
		double xy = q.x * q.y;
		double xz = q.x * q.z;
		double yz = q.y * q.z;
		double wx = q.w * q.x;
		double wy = q.w * q.y;
		double wz = q.w * q.z;

		double[][] m = new double[3][3];
		m[0][0] = 1 - 2 * (yy + zz);
		m[0][1] = 2 * (xy - wz);
		m[0][2] = 2 * (xz + wy);
		m[1][0] = 2 * (xy + wz);
		m[1][1] = 1 - 2 * (xx + zz);
		m[1][2] = 2 * (yz - wx);
		m[2][0] = 2 * (xz - wy);
		m[2][1] = 2 * (yz + wx);
		m[2][2] = 1 - 2 * (xx + yy);
		return new Matrix3d(m);
	}

	public Rotation2d toRotation2d() {
		return new Rotation2d(z);
	}

	public ca.team4308.absolutelib.math.Vector3 toVector() {
		ca.team4308.absolutelib.math.Vector3 axis = getAxis();
		double angle = getAngle();
		return new ca.team4308.absolutelib.math.Vector3(axis.x * angle, axis.y * angle, axis.z * angle);
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj) {
			return true;
		}
		if (!(obj instanceof Rotation3d)) {
			return false;
		}
		Rotation3d other = (Rotation3d) obj;
		return Double.compare(x, other.x) == 0
				&& Double.compare(y, other.y) == 0
				&& Double.compare(z, other.z) == 0;
	}

	@Override
	public int hashCode() {
		int result = Double.hashCode(x);
		result = 31 * result + Double.hashCode(y);
		result = 31 * result + Double.hashCode(z);
		return result;
	}

	@Override
	public String toString() {
		return String.format("Rotation3d{x=%.6f, y=%.6f, z=%.6f}", x, y, z);
	}

	private Quaternion toQuaternion() {
		double cx = Math.cos(x / 2.0);
		double sx = Math.sin(x / 2.0);
		double cy = Math.cos(y / 2.0);
		double sy = Math.sin(y / 2.0);
		double cz = Math.cos(z / 2.0);
		double sz = Math.sin(z / 2.0);

		double w = cz * cy * cx + sz * sy * sx;
		double qx = cz * cy * sx - sz * sy * cx;
		double qy = cz * sy * cx + sz * cy * sx;
		double qz = sz * cy * cx - cz * sy * sx;
		return new Quaternion(w, qx, qy, qz);
	}

	private static Rotation3d fromQuaternion(Quaternion q) {
		q = q.normalize();
		double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
		double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
		double roll = Math.atan2(sinr_cosp, cosr_cosp);

		double sinp = 2.0 * (q.w * q.y - q.z * q.x);
		double pitch;
		if (Math.abs(sinp) >= 1.0) {
			pitch = Math.copySign(Math.PI / 2.0, sinp);
		} else {
			pitch = Math.asin(sinp);
		}

		double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
		double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
		double yaw = Math.atan2(siny_cosp, cosy_cosp);
		return new Rotation3d(roll, pitch, yaw);
	}

	private static Rotation3d fromAxisAngle(ca.team4308.absolutelib.math.Vector3 axis, double angle) {
		double norm = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
		if (norm == 0.0) {
			return new Rotation3d();
		}
		double ux = axis.x / norm;
		double uy = axis.y / norm;
		double uz = axis.z / norm;

		double halfAngle = angle / 2.0;
		double sinHalf = Math.sin(halfAngle);
		Quaternion q = new Quaternion(Math.cos(halfAngle), ux * sinHalf, uy * sinHalf, uz * sinHalf);
		return fromQuaternion(q);
	}

	private static double clamp(double value, double min, double max) {
		if (value < min) {
			return min;
		}
		if (value > max) {
			return max;
		}
		return value;
	}

	public static class Angle {
		private final double radians;

		public Angle(double radians) {
			this.radians = radians;
		}

		public double toRadians() {
			return radians;
		}

		public double toDegrees() {
			return Math.toDegrees(radians);
		}

		@Override
		public String toString() {
			return String.format("Angle{%.6f rad}", radians);
		}
	}

	public static class Quaternion {
		private final double w;
		private final double x;
		private final double y;
		private final double z;

		public Quaternion(double w, double x, double y, double z) {
			this.w = w;
			this.x = x;
			this.y = y;
			this.z = z;
		}

		public Quaternion multiply(Quaternion other) {
			double nw = w * other.w - x * other.x - y * other.y - z * other.z;
			double nx = w * other.x + x * other.w + y * other.z - z * other.y;
			double ny = w * other.y - x * other.z + y * other.w + z * other.x;
			double nz = w * other.z + x * other.y - y * other.x + z * other.w;
			return new Quaternion(nw, nx, ny, nz);
		}

		public Quaternion inverse() {
			double normSq = w * w + x * x + y * y + z * z;
			if (normSq == 0.0) {
				return new Quaternion(1.0, 0.0, 0.0, 0.0);
			}
			return new Quaternion(w / normSq, -x / normSq, -y / normSq, -z / normSq);
		}

		public Quaternion normalize() {
			double norm = Math.sqrt(w * w + x * x + y * y + z * z);
			if (norm == 0.0) {
				return new Quaternion(1.0, 0.0, 0.0, 0.0);
			}
			return new Quaternion(w / norm, x / norm, y / norm, z / norm);
		}

		public Quaternion slerp(Quaternion other, double t) {
			double cosHalfTheta = w * other.w + x * other.x + y * other.y + z * other.z;
			Quaternion end = other;
			if (cosHalfTheta < 0.0) {
				end = new Quaternion(-other.w, -other.x, -other.y, -other.z);
				cosHalfTheta = -cosHalfTheta;
			}

			if (Math.abs(cosHalfTheta) >= 1.0) {
				return new Quaternion(w, x, y, z);
			}

			double halfTheta = Math.acos(cosHalfTheta);
			double sinHalfTheta = Math.sqrt(1.0 - cosHalfTheta * cosHalfTheta);

			if (Math.abs(sinHalfTheta) < 1e-9) {
				return new Quaternion(
						w * 0.5 + end.w * 0.5,
						x * 0.5 + end.x * 0.5,
						y * 0.5 + end.y * 0.5,
						z * 0.5 + end.z * 0.5);
			}

			double ratioA = Math.sin((1 - t) * halfTheta) / sinHalfTheta;
			double ratioB = Math.sin(t * halfTheta) / sinHalfTheta;
			return new Quaternion(
					w * ratioA + end.w * ratioB,
					x * ratioA + end.x * ratioB,
					y * ratioA + end.y * ratioB,
					z * ratioA + end.z * ratioB
			).normalize();
		}

		@Override
		public String toString() {
			return String.format("Quaternion{w=%.6f, x=%.6f, y=%.6f, z=%.6f}", w, x, y, z);
		}
	}

	public static class Rotation2d {
		private final double radians;

		public Rotation2d() {
			this(0.0);
		}

		public Rotation2d(double radians) {
			this.radians = radians;
		}

		public double getRadians() {
			return radians;
		}

		public double getDegrees() {
			return Math.toDegrees(radians);
		}

		@Override
		public String toString() {
			return String.format("Rotation2d{%.6f rad}", radians);
		}
	}

	public static class Matrix3d {
		private final double[][] data;

		public Matrix3d(double[][] data) {
			if (data == null || data.length != 3 || data[0].length != 3 || data[1].length != 3 || data[2].length != 3) {
				throw new IllegalArgumentException("Matrix3d requires a 3x3 array");
			}
			this.data = new double[3][3];
			for (int i = 0; i < 3; i++) {
				System.arraycopy(data[i], 0, this.data[i], 0, 3);
			}
		}

		public double get(int row, int col) {
			return data[row][col];
		}

		@Override
		public String toString() {
			return String.format("Matrix3d{[%.3f, %.3f, %.3f],[%.3f, %.3f, %.3f],[%.3f, %.3f, %.3f]}",
					data[0][0], data[0][1], data[0][2],
					data[1][0], data[1][1], data[1][2],
					data[2][0], data[2][1], data[2][2]);
		}
	}
    
}
