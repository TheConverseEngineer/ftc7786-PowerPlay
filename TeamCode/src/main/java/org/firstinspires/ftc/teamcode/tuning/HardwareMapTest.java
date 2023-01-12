package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

// Tests all hardware maps to make sure everything is configured properly
@TeleOp(name = "HardwareMap Test", group = "default")
public class HardwareMapTest extends LinearOpMode {

    DcMotor lf, rf, lr, rr;
    VoltageSensor voltage;

    DcMotor lift1, lift2;

    DcMotor armMotor;
    Servo fbServo, flipServo, rightGripper, leftGripper;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rf = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        lr = hardwareMap.get(DcMotor.class, "leftreardrive");
        rr = hardwareMap.get(DcMotor.class, "rightreardrive");
        telemetry.addData("Drive motor status", "initialized");
        telemetry.update();

        voltage = hardwareMap.voltageSensor.iterator().next();
        telemetry.addData("Voltage sensor status", "initialized");
        telemetry.update();

        lift1 = hardwareMap.get(DcMotor.class, "liftdrive1");
        lift2 = hardwareMap.get(DcMotor.class, "liftdrive2");
        telemetry.addData("Lift motor status", "initialized");
        telemetry.update();

        armMotor = hardwareMap.get(DcMotor.class, "armdrive");
        fbServo = hardwareMap.get(Servo.class, "fbservo");
        flipServo = hardwareMap.get(Servo.class, "flipservo");
        rightGripper = hardwareMap.get(Servo.class, "rightgripper");
        leftGripper = hardwareMap.get(Servo.class, "leftgripper");
        telemetry.addData("Arm status", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("elevator pos", lift1.getCurrentPosition());
            telemetry.addData("arm pos", armMotor.getCurrentPosition());
            telemetry.addData("left odo pod", lf.getCurrentPosition());
            telemetry.addData("right odo pod", rr.getCurrentPosition());
            telemetry.addData("center odo pod", rf.getCurrentPosition());
            telemetry.update();
        }





    }
}
