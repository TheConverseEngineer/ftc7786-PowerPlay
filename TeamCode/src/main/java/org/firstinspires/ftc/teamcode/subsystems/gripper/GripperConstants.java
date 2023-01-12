package org.firstinspires.ftc.teamcode.subsystems.gripper;

import com.acmerobotics.dashboard.config.Config;

@Config
public class GripperConstants {
    // Hardware map Constants
    public static final String leftGripServoName = "leftGripServo";
    public static final String rightGripServoName = "rightGripServo";

    // Positions
    public static final double leftGripClosePos = 0;
    public static final double rightGripClosePos = 0;
    public static final double leftGripOpenPos = 0;
    public static final double rightGripOpenPos = 0;

    // Defaults
    public static final double leftGripDefaultPos = leftGripClosePos;
    public static final double rightGripDefaultPos = rightGripClosePos;

    // Offset value for the gripper servos
    public static final double gripServoOffsetVal = 0;
}
