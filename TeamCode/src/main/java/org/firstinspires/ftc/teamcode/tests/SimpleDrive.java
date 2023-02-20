package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "simple drive", group = "default")
public class SimpleDrive extends LinearOpMode {
    DcMotor lf, rf, lr, rr;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rf = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        lr = hardwareMap.get(DcMotor.class, "leftreardrive");
        rr = hardwareMap.get(DcMotor.class, "rightreardrive");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            lf.setPower(gamepad1.left_stick_y);
            rf.setPower(-gamepad1.left_stick_y);
            lr.setPower(gamepad1.left_stick_y);
            rr.setPower(-gamepad1.left_stick_y);

        }
    }
}
