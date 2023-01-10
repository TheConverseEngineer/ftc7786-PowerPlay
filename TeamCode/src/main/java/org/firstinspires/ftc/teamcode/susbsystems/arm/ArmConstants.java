package org.firstinspires.ftc.teamcode.susbsystems.arm;

public class ArmConstants {
    //Hardwaremap names
    public static final String ARM_MOTOR_NAME = "armMotor";
    public static final String FOURBAR_SERVO_NAME = "fourbarServo";
    public static final String FLIP_SERVO_NAME = "flipServo";

    //constants for manual adjust offset
    public static final double FOURBAR_SERVO_OFFSET = 0.01;
    public static final double FLIP_SERVO_OFFSET = 0.01;
    public static final double ARM_MOTOR_OFFSET = 0.01;


    //Arm Positions
    public static final double LOW_POS = 0;
    public static final double MID_POS = 0;
    public static final double HIGH_POS = 0;
    public static final double ARM_DEFAULT_POS = 0;

    //Flip Servo Positions
    public static final double FLIP_UP = 0;
    public static final double FLIP_DOWN = 0;

    //Fourbar Servo Positions
    public static final double FOURBAR_FRONT_DOWN = 0;
    public static final double FOURBAR_FRONT_UP = 0;
    public static final double FOURBAR_BACK_DOWN = 0;
    public static final double FOURBAR_BACK_UP = 0;
    public static final double FOURBAR_MID = 0;

    //PID Tuning Values
    public static double P_PID = 0;
    public static double I_PID = 0;
    public static double D_PID = 0;
    public static double I_CAP = 0;
}
