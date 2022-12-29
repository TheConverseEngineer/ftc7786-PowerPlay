package org.firstinspires.ftc.teamcode.susbsystems;

public class LiftStateManager {

    double currentPos;

    public LiftStateManager(double initialPos) {
        currentPos = initialPos;
    }

    /** Sets the new target position */
    public void setTargetPos(double newTarget) {
        currentPos = newTarget;
    }

    /** Gets the current target position of the system (updated dynamically with a built-in timer)
     *
     * @return  the current target position
     */
    public double getS() {
        return currentPos;
    }

    /** Gets the current target position of the system (updated dynamically with a built-in timer)
     *
     * @return  the current target velocity
     */
    public double getV() {
        return 0;
    }

    /** Gets the current target position of the system (updated dynamically with a built-in timer)
     *
     * @return  the current target acceleration
     */
    public double getA() {
        return 0;
    }
}
