package ca.team4308.absolutelib.math;

/**
 * Represents a 3D vector with x, y, and z components.
 * Provides basic vector operations and utilities.
 */
public class Vector3 {
    public double x;
    public double y;
    public double z;

    /**
     * Creates a zero vector (0,0,0).
     */
    public Vector3() {
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
    }

    /**
     * Creates a vector with all components set to the same value.
     * @param d The value to set for all components
     */
    public Vector3(double d) {
        this.x = d;
        this.y = d;
        this.z = d;
    }

    /**
     * Creates a vector with specified x, y, and z components.
     * @param x The x component
     * @param y The y component
     * @param z The z component
     */
    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Adds another vector to this vector.
     * @param other The vector to add
     * @return A new Vector3 containing the sum
     */
    public Vector3 add(Vector3 other) {
        return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    /**
     * Subtracts another vector from this vector.
     * @param other The vector to subtract
     * @return A new Vector3 containing the difference
     */
    public Vector3 sub(Vector3 other) {
        return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    /**
     * Multiplies this vector component-wise with another vector.
     * @param other The vector to multiply by
     * @return A new Vector3 containing the product
     */
    public Vector3 mul(Vector3 other) {
        return new Vector3(this.x * other.x, this.y * other.y, this.z * other.z);
    }

    /**
     * Divides this vector component-wise by another vector.
     * @param other The vector to divide by
     * @return A new Vector3 containing the quotient
     */
    public Vector3 div(Vector3 other) {
        return new Vector3(this.x / other.x, this.y / other.y, this.z / other.z);
    }

    /**
     * Checks if this vector equals another vector.
     * @param other The vector to compare with
     * @return true if the vectors are equal, false otherwise
     */
    public boolean equals(Vector3 other) {
        return (this.x == other.x && this.y == other.y && this.z == other.z);
    }

    /**
     * Calculates the magnitude (length) of this vector.
     * @return The magnitude of the vector
     */
    public double magnitude() {
        return Math.sqrt((x * x) + (y * y) + (z * z));
    }

    /**
     * Normalizes this vector to have a magnitude of 1.
     * @return This vector after normalization
     */
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

    /**
     * Normalizes this vector to have a specified maximum magnitude.
     * @param max The maximum magnitude to normalize to
     * @return This vector after normalization
     */
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
