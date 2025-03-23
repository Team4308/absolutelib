package ca.team4308.absolutelib.math;

/**
 * Represents a 2D vector with x and y components.
 * Provides basic vector operations and utilities.
 */
public class Vector2 {
    public double x;
    public double y;

    /**
     * Creates a zero vector (0,0).
     */
    public Vector2() {
        this.x = 0.0;
        this.y = 0.0;
    }

    /**
     * Creates a vector with both components set to the same value.
     * @param d The value to set for both components
     */
    public Vector2(double d) {
        this.x = d;
        this.y = d;
    }

    /**
     * Creates a vector with specified x and y components.
     * @param x The x component
     * @param y The y component
     */
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Adds another vector to this vector.
     * @param other The vector to add
     * @return A new Vector2 containing the sum
     */
    public Vector2 add(Vector2 other) {
        return new Vector2(this.x + other.x, this.y + other.y);
    }

    /**
     * Subtracts another vector from this vector.
     * @param other The vector to subtract
     * @return A new Vector2 containing the difference
     */
    public Vector2 sub(Vector2 other) {
        return new Vector2(this.x - other.x, this.y - other.y);
    }

    /**
     * Multiplies this vector component-wise with another vector.
     * @param other The vector to multiply by
     * @return A new Vector2 containing the product
     */
    public Vector2 mul(Vector2 other) {
        return new Vector2(this.x * other.x, this.y * other.y);
    }

    /**
     * Divides this vector component-wise by another vector.
     * @param other The vector to divide by
     * @return A new Vector2 containing the quotient
     */
    public Vector2 div(Vector2 other) {
        return new Vector2(this.x / other.x, this.y / other.y);
    }

    /**
     * Checks if this vector equals another vector.
     * @param other The vector to compare with
     * @return true if the vectors are equal, false otherwise
     */
    public boolean equals(Vector2 other) {
        return (this.x == other.x && this.y == other.y);
    }

    /**
     * Calculates the magnitude (length) of this vector.
     * @return The magnitude of the vector
     */
    public double magnitude() {
        return Math.sqrt((x * x) + (y * y));
    }

    /**
     * Normalizes this vector to have a magnitude of 1.
     * @return This vector after normalization
     */
    public Vector2 normalize() {
        double length = Math.sqrt(x * x + y * y);

        if (length != 0.0) {
            double s = 1.0 / length;
            x = x * s;
            y = y * s;
        }

        return this;
    }

    /**
     * Normalizes this vector to have a specified maximum magnitude.
     * @param max The maximum magnitude to normalize to
     * @return This vector after normalization
     */
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
