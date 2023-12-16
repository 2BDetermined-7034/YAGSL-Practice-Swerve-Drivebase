package frc.robot.utils;

public class VectorAngle {
    private double deadZone;
    private double previousAngle = 0.0;
    public VectorAngle() {
        this.deadZone = 0.75;
    }
    public VectorAngle(double deadZone) {
        this.deadZone = deadZone;
    }

    public double getAngle(Vector2 vector) {
        if (vector.magnitude() > deadZone) {
            previousAngle = Math.atan2(vector.y, vector.x);
        }

        return previousAngle;
    }
    public void setDeadZone(double deadZone) {
        this.deadZone = deadZone;
    }
}
