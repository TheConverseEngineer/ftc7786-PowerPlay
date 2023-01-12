package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElevatorConstants {

    public static double ELEVATOR_P;
    public static double ELEVATOR_kStatic;
    public static double ELEVATOR_kV;

    public static double ELEVATOR_MAX_V = 30;
    public static double ELEVATOR_MAX_A = 30;

    public static double TICKS_PER_INCH = 76.4938445235;

    public static double MIN_POS = 0;
    public static double MAX_POS = 33;
}
