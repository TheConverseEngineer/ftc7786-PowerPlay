package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {

    // Odometry values (inches unless specified)
    public static double TICKS_PER_INCH = 4890;
    public static double TRACK_DIAMETER = 6;
    public static double LATERAL_RADIUS = 3;
    public static double LATERAL_MULTIPLIER = 1.11; // This value is most likely already tuned

    // Trajectory construction values (inches and radians)
    public static double PROFILE_RESOLUTION = .25;
    public static double MAX_VELOCITY = 30;
    public static double MAX_ACCELERATION = 30;
    public static double MAX_ANGULAR_VELOCITY = Math.PI / 3;

    // Motor feedforward values
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    // Proportional follower controller
    public static double kPLateral = 0;
    public static double kPRotational = 0;
}
