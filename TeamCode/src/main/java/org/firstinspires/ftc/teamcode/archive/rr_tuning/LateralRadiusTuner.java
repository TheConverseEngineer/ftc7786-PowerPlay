package org.firstinspires.ftc.teamcode.archive.rr_tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.archive.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.AutoDriveV3;

@TeleOp(name = "lateral tuner", group = "rr")
@Disabled
public class LateralRadiusTuner extends LinearOpMode {
    DcMotor right, left, rear, rm, lr;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        right = hardwareMap.get(DcMotor.class, "rightencoder");
        rear = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        rm = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        lr = hardwareMap.get(DcMotor.class, "leftfrontdrive");


        right.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("status", "ready");
        telemetry.update();

        waitForStart();

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("do", " 15 spins counter-clockwise");
            telemetry.addData("tracked dist", right.getCurrentPosition() - left.getCurrentPosition());
            telemetry.addData("angle", (right.getCurrentPosition() - left.getCurrentPosition()) / (AutoDriveV3.TICKS_PER_INCH * AutoDriveV3.TRACK_DIAMETER));
            double power = gamepad1.right_stick_x;
            left.setPower(power);
            rm.setPower(-power);
            lr.setPower(power);
            rear.setPower(-power);
            telemetry.update();

            double heading = (right.getCurrentPosition() - left.getCurrentPosition()) / (DriveConstants.TRACK_DIAMETER * DriveConstants.TICKS_PER_INCH);
            if (Math.abs(heading) > 0.001)
                telemetry.addData("lateral radius", rear.getCurrentPosition() / (heading));
            telemetry.update();
        }
    }
}
