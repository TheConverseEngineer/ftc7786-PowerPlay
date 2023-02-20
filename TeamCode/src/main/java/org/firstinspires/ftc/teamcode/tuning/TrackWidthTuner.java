package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "track width", group = "tuning")
public class TrackWidthTuner extends LinearOpMode {
    DcMotor right, left, rear, rm, lr;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        right = hardwareMap.get(DcMotor.class, "rightencoder");
        rear = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        rm = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        lr = hardwareMap.get(DcMotor.class, "leftfrontdrive");


        right.setDirection(DcMotorSimple.Direction.REVERSE);
        rear.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("status", "ready");
        telemetry.update();

        waitForStart();

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!gamepad1.a && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("do", " 15 spins counter-clockwise");
            telemetry.addData("tracked dist", right.getCurrentPosition() - left.getCurrentPosition());
            double power = gamepad1.right_stick_x;
            left.setPower(power);
            rm.setPower(-power);
            lr.setPower(power);
            rear.setPower(-power);
            telemetry.update();
        }

        telemetry.addData("track width", (right.getCurrentPosition() - left.getCurrentPosition()) / (30*Math.PI));
        telemetry.addData("left", left.getCurrentPosition());
        telemetry.addData("right", right.getCurrentPosition());
        telemetry.addData("lateral radius", rear.getCurrentPosition() / (30*Math.PI));
        telemetry.update();
        while (!isStopRequested() && opModeIsActive()) {

        }
    }
}
