package ca.team4308.absolutelib.math;

public class Vector3 {
    public double x;
    public double y;
    public double z;

    public Vector3() {
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
    }

    public Vector3(double d) {
        this.x = d;
        this.y = d;
        this.z = d;
    }

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3 add(Vector3 other) {
        return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    public Vector3 sub(Vector3 other) {
        return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    public Vector3 mul(Vector3 other) {
        return new Vector3(this.x * other.x, this.y * other.y, this.z * other.z);
    }

    public Vector3 div(Vector3 other) {
        return new Vector3(this.x / other.x, this.y / other.y, this.z / other.z);
    }

    public boolean equals(Vector3 other) {
        return (this.x == other.x && this.y == other.y && this.z == other.z);
    }

    public double magnitude() {
        return Math.sqrt((x * x) + (y * y) + (z * z));
    }

    public Vector3 normalize() {
        double length = magnitude();

        if (length != 0.0) {
            double s = 1.0 / length;
            x = x * s;
            y = y * s;
            z = z * s;
        }

        return this;
    }

    public Vector3 normalize(double max) {
        double length = magnitude();

        if (length != 0.0) {
            double s = max / length;
            x = x * s;
            y = y * s;
            z = z * s;
        }

        return this;
    }
}
