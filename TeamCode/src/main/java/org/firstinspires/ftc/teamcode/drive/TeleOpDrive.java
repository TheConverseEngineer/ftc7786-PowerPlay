package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.command.Subsystem;

public class TeleOpDrive implements Subsystem {

    DcMotor leftFront, rightFront, leftRear, rightRear;


    public TeleOpDrive(DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
    }

    /** Method to set drivetrain velocity using a joystick (just feed values directly)
     * Will automatically deal with inverted motors
     *
     * @param x     the gamepad x value
     * @param y     the gamepad y value
     * @param rx    the gamepad right-stick x value
     */
    public void gamepadControl(double x, double y, double rx) {
        y *= -1; // Y stick is inverted on gamepad
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double maxDispPower = Math.max(Math.abs(cos), Math.abs(sin));

        double lf, rf, lr, rr;
        lf = power * cos / maxDispPower + rx;
        rf = power * sin / maxDispPower - rx;
        lr = power * sin / maxDispPower + rx;
        rr = power * cos / maxDispPower - rx;

        if (power + Math.abs(rx) > 1) {
            lf /= power + Math.abs(rx);
            rf /= power + Math.abs(rx);
            lr /= power + Math.abs(rx);
            rr /= power + Math.abs(rx);
        }

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }

    @Override
    public void periodic() {

    }
}
