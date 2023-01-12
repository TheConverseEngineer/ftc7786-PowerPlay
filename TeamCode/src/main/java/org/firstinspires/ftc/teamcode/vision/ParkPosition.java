package org.firstinspires.ftc.teamcode.vision;

public enum ParkPosition {
    LEFT, CENTER, RIGHT;

    /** Maps the detected april tag code to an enum value */
    public static ParkPosition aprilCodeToPosition(int code) {
        switch (code) {
            case 0: return RIGHT;
            case 1: return CENTER;
            default: return LEFT;
        }
    }
}
