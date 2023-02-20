package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {

    public static double ARM_MAX_POS = 1405;
    public static double ARM_MIN_POS = 0;

    public static double MAX_ARM_POWER = 0.6;
    public static double MIN_ARM_POWER = -0.3;

    // Servo stuff
    public static double FOUR_BAR_SERVO_FORWARD = 0.15;
    public static double FOUR_BAR_SERVO_REVERSE = 0.65;
    public static double FLIP_SERVO_FORWARD = 0.3;
    public static double FLIP_SERVO_REVERSE = 1;

    // PID Tuning Values
    public static double P_PID_UP = 0.01;
    public static double P_PID_DOWN = 0.001;
    public static double MIN_END_SPEED = 0.015;
    public static double I_PID = 0.002;
    public static double D_PID = 0;
    public static double I_CAP = 0.05;
    public static double STATIC_PID = 0.1;
}
