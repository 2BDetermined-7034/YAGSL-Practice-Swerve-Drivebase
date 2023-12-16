package frc.robot.utils;

import java.util.ArrayList;

public class RationalInterp {
    public RationalInterp() {}

    /*
    Be sure to pass in the points with the X coordinate in ascending order.
     */
    public RationalInterp(Vector2 points[]) {
        this.points = points;
    }

    public double get(double x) {
        Vector2 a, b;

        a = new Vector2(points[0]);
        b = new Vector2(points[points.length - 1]);

        // int end = points.length - 1;
        // if (x > points[end].x) {
        //     a = new Vector2(end - 1);
        //     b = new Vector2(end);
        // } else if (x > points[0].x) {
        //     for (int i = 0; i < end; ++i) {
        //         if (x < points[i + 1].x) {
        //             System.out.println(x);
        //             System.out.println(i);
        //             a = new Vector2(points[i]);
        //             b = new Vector2(points[i + 1]);
                    
        //             break;
        //         }
        //     }
        // }

        return getFromRLerp(x, a, b);
    }

    private double getFromRLerp(double x, Vector2 a, Vector2 b) {
        a.y = 1.0 / a.y;
        b.y = 1.0 / b.y;

        double result;
        result = x - a.x;
        result *= (b.y - a.y) / (b.x - a.x);
        result += a.y;

        result = 1.0 / result;

        return result;
    }

    public Vector2 points[];
}
