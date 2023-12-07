package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Vector2 {
    public double x, y;

    public Vector2() {}

    public Vector2(double x, double y) {
        this.x = x; this.y = y;
    }

    public Vector2(double v) {
        this.x = v; this.y = v;
    }

    public Vector2(Translation2d vector) {
        this.x = vector.getX();
        this.y = vector.getY();
    }

    public Vector2(Vector2 vector) {
        this.x = vector.x;
        this.y = vector.y;
    }

    public Translation2d toTranslation2d() {
        return new Translation2d(x, y);
    }

    public Vector2 add(Vector2 vector) {
        this.x += vector.x; this.y += vector.y;
        return this;
    }
    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    public Vector2 sub(Vector2 vector) {
        this.x -= vector.x; this.y -= vector.y;
        return this;
    }
    public static Vector2 sub(Vector2 a, Vector2 b) {
        return new Vector2(a.x - b.x, a.y - b.y);
    }

    public Vector2 mul(Vector2 vector) {
        this.x *= vector.x; this.y *= vector.y;
        return this;
    }
    public Vector2 mul(double value) {
        this.x *= value; this.y *= value;
        return this;
    }
    public static Vector2 mul(Vector2 a, Vector2 b) {
        return new Vector2(a.x * b.x, a.y * b.y);
    }
    public static Vector2 mul(Vector2 vector, double value) {
        return new Vector2(vector.x * value, vector.y * value);
    }

    public Vector2 div(Vector2 vector) {
        this.x /= vector.x; this.y /= vector.y;
        return this;
    }
    public Vector2 div(double value) {
        this.x /= value; this.y /= value;
        return this;
    }
    public static Vector2 div(Vector2 a, Vector2 b) {
        return new Vector2(a.x / b.x, a.y / b.y);
    }
    public static Vector2 div(Vector2 vector, double value) {
        return new Vector2(vector.x / value, vector.y / value);
    }

    double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    double normalize() {
        double magnitude = this.magnitude();
        double invMagnitude = 1.0 / magnitude;

        this.x *= invMagnitude;
        this.y *= invMagnitude;

        return magnitude;
    }

    double dotProduct(Vector2 vector) {
        return this.x * vector.x + this.y * vector.y;
    }
}
