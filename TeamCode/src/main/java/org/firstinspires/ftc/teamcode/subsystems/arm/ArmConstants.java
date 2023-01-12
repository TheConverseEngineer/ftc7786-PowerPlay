package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {

    public static final double ARM_MAX_POS = 360;
    public static final double ARM_MIN_POS = 0;

    // Servo stuff
    public static final double FOUR_BAR_SERVO_FORWARD = 0;
    public static final double FOUR_BAR_SERVO_REVERSE = 1;
    public static final double FLIP_SERVO_FORWARD = 0;
    public static final double FLIP_SERVO_REVERSE = 1;

    // PID Tuning Values
    public static double P_PID = 0;
    public static double I_PID = 0;
    public static double D_PID = 0;
    public static double I_CAP = 0;
}
