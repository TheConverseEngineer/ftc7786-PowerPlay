package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.MutablePose2d;

@Disabled
@TeleOp(name = "TPI Test", group = "tuning")
public class TPITest extends LinearOpMode {
    DcMotor left, right;
    MutablePose2d pose;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        right = hardwareMap.get(DcMotor.class, "rightencoder");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long trackedDist = 0;
        while (!gamepad1.a && !isStopRequested() & opModeIsActive()) {
            trackedDist = left.getCurrentPosition() + right.getCurrentPosition();
            telemetry.addData("Tracked Distance", trackedDist);
            telemetry.addData("After moving 80 inches, press", "a");
            telemetry.update();
        }

        telemetry.addData("Ticks-Per-Inch", (double)trackedDist/160d);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {
            // Wait for opmode to end
        }
    }
}
