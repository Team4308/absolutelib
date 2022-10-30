package ca.team4308.absolutelib.math;

public class Vector2 {
    public double x;
    public double y;

    public Vector2() {
        this.x = 0.0;
        this.y = 0.0;
    }

    public Vector2(double d) {
        this.x = d;
        this.y = d;
    }

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2 add(Vector2 other) {
        return new Vector2(this.x + other.x, this.y + other.y);
    }

    public Vector2 sub(Vector2 other) {
        return new Vector2(this.x - other.x, this.y - other.y);
    }

    public Vector2 mul(Vector2 other) {
        return new Vector2(this.x * other.x, this.y * other.y);
    }

    public Vector2 div(Vector2 other) {
        return new Vector2(this.x / other.x, this.y / other.y);
    }

    public boolean equals(Vector2 other) {
        return (this.x == other.x && this.y == other.y);
    }

    public double magnitude() {
        return Math.sqrt((x * x) + (y * y));
    }

    public Vector2 normalize() {
        double length = Math.sqrt(x * x + y * y);

        if (length != 0.0) {
            double s = 1.0 / length;
            x = x * s;
            y = y * s;
        }

        return this;
    }

    public Vector2 normalize(double max) {
        double length = Math.sqrt(x * x + y * y);

        if (length != 0.0) {
            double s = max / length;
            x = x * s;
            y = y * s;
        }

        return this;
    }
}
