package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElevatorConstants {

    public static double ELEVATOR_P = 0.4;
    public static double ELEVATOR_I = 0.5;
    public static double ELEVATOR_kStatic = 0.08;
    public static double ELEVATOR_MAX_I = .2;

    // Theoretical: 0.0274405074
    public static double ELEVATOR_kV_POSITIVE = 0.04;
    public static double ELEVATOR_kV_NEGATIVE = 0.03;

    public static double ELEVATOR_MAX_V = 30;
    public static double ELEVATOR_MAX_A = 40;
    public static double ELEVATOR_MAX_P_VEL = 0.45;
    public static double ELEVATOR_MAX_MOTOR_POWER = 0.85;

    public static double TICKS_PER_INCH = 76.4938445235;

    public static double MIN_POS = -0.05;
    public static double MAX_POS = 18.5;

}
