package org.firstinspires.ftc.teamcode.utils;

/** A mutable version of Pose2d, based off of roadrunner's implementation */
public class MutablePose2d {
    public double x, y, theta;

    public MutablePose2d(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    /** Integrates local displacement over time and adds it to the pose */
    public void integrateLocalDisp(double dx, double dy, double dTheta) {
        double a = Math.cos(theta);
        double b = Math.sin(theta);
        if (dTheta < 0.000001) {
            this.x += a*dx - b*dy;
            this.y += b*dx + a*dy;
        } else {
            double c = Math.sin(dTheta) / dTheta;
            double d = (1-Math.cos(dTheta)) / dTheta;
            this.x += dx*(a*c - b*d) - dy*(a*d + b*c);
            this.y += dx*(b*c - a*d) + dy*(b*d + a*c);
        }
        this.theta += dTheta;
    }
}
