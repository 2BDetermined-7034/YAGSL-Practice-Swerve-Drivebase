package frc.robot.utils;

public class Distance {
    private final double zNear;
    private final double vFOVScale;
    private final double scaleCoefficient;
    private final double heightSquared;
    public Distance(double height, double scale, double zNear, double hFOV, double vFOV) {
        this.zNear = zNear;
        this.vFOVScale = 1.0 / vFOV;

        heightSquared = height * height;
        scaleCoefficient = height / Math.tan(hFOV * 0.5) * scale;
    }

    public double getZNear() { return zNear; }
    public double getHeight() { return Math.sqrt(heightSquared); }
    public double getFOV() {
        return Math.atan(getHeight() / scaleCoefficient) * 2.0;
    }

    public double getDistance(double x, double y) {
        y *= vFOVScale;

        //Add a 1.0 / cos(x) term to remove fisheye. (OPTIONAL)
        double z = scaleCoefficient / (y /* * Math.cos(x) */) - zNear;
        x *= z;

        return Math.sqrt(/* heightSquared + */ z * z + x * x);
    }
}
