package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.MutablePose2d;

@TeleOp(name = "TPI Test", group = "tuning")
public class TPITest extends LinearOpMode {
    DcMotor left, right;
    MutablePose2d pose;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        right = hardwareMap.get(DcMotor.class, "rightreardrive");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double trackedDist = 0;
        while (!gamepad1.a && !isStopRequested() & opModeIsActive()) {
            trackedDist = ((double)left.getCurrentPosition() + right.getCurrentPosition()) / 2;
            telemetry.addData("Tracked Distance", trackedDist);
            telemetry.addData("After moving 80 inches, press", "a");
            telemetry.update();
        }

        telemetry.addData("Ticks-Per-Inch", trackedDist/80);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {
            // Wait for opmode to end
        }
    }
}
