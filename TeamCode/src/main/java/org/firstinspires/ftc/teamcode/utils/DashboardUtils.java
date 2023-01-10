package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class DashboardUtils {

    // Suppress instantiation
    private DashboardUtils() { }

    /** Draw a robot on the canvas (a circle with a line representing it's heading */
    public static void drawRobot(MutablePose2d robotPose, String color, Canvas canvas) {
        canvas.setStroke(color);
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(robotPose.x, robotPose.y, 5);
        canvas.strokeLine(robotPose.x, robotPose.y,
                robotPose.x + 5*Math.cos(robotPose.theta),
                robotPose.y + 5*Math.sin(robotPose.theta));
    }

    public static void drawRobot(Pose2d robotPose, String color, Canvas canvas) {
        canvas.setStroke(color);
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(robotPose.getX(), robotPose.getY(), 5);
        canvas.strokeLine(robotPose.getX(), robotPose.getY(),
                robotPose.getX() + 5*Math.cos(robotPose.getHeading()),
                robotPose.getY() + 5*Math.sin(robotPose.getHeading()));
    }

}
