package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.utils.MathUtils;

/** Once-mutable data class that stores the located position of the vision cone */
public class DetectionStatus {

    private ParkPosition pos;
    private boolean mutable;

    public DetectionStatus() {
        mutable = true;
        pos = ParkPosition.CENTER;
    }

    /** Sets the detected position (can only be called once) */
    public void setPosition(ParkPosition pos) {
        // if (!this.mutable) return;

        this.pos = pos;
        this.mutable = false;
    }

    /** Returns the detected park position */
    public ParkPosition getPos() {
        return pos;
    }

    /** Makes this class immutable */
    public void lock() {
        mutable = false;
    }
}
