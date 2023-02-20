package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.archive.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.subsystems.gripper.Gripper;

@Config
@TeleOp(name = "TeleOp", group = "default")
public class ElevatorTuner extends LinearOpMode {
    DcMotorEx lift1, lift2, armMotor;
    Elevator elevator;
    Arm2 arm;
    Gripper gripper;
    ServoImplEx armServo, flipServo, gripperRight, gripperLeft;

    double lastTarget = 0;
    public static double target = 0;

    public static double ARM_ADJ_RATE = 7;

    double lastArmTarget = 0;
    public static double armTarget = 0;

    private boolean g2aT = false, g2bT = false, g2yT = false, g2xT = false, g1lbT = false;
    private boolean gripped = true;

    private double headOff = Math.PI/2;

    DcMotor lf, rf, lr, rr, re;


    @Override
    public void runOpMode() throws InterruptedException {
        lift1 = hardwareMap.get(DcMotorEx.class, "liftdrive1");
        lift2 = hardwareMap.get(DcMotorEx.class, "liftdrive2");
        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armServo = hardwareMap.get(ServoImplEx.class, "armservo");
        armServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        flipServo = hardwareMap.get(ServoImplEx.class, "flipservo");
        flipServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        gripperLeft = hardwareMap.get(ServoImplEx.class, "gripperleft");
        gripperRight = hardwareMap.get(ServoImplEx.class, "gripperright");
        gripperLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gripperRight.setPwmRange(new PwmControl.PwmRange(500, 2500));

        lf = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rf = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        lr = hardwareMap.get(DcMotor.class, "leftreardrive");
        rr = hardwareMap.get(DcMotor.class, "rightreardrive");
        re = hardwareMap.get(DcMotor.class, "rightencoder");

        elevator = new Elevator(lift1, lift2, 0);
        arm = new Arm2(armMotor, 0);
        gripper = new Gripper(gripperLeft, gripperRight);

        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        re.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        re.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry = FtcDashboard.getInstance().getTelemetry();

        waitForStart();

        armServo.setPosition(0.1);
        flipServo.setPosition(0.35);

        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        re.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gripper.close();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad2.right_bumper) {
                if (elevator.getPosition() > 16) arm.setTargetPos(915);
                else arm.setTargetPos(950);
            } else if (gamepad2.left_bumper) {
                arm.setTargetPos(0);
            }

            if (gamepad2.y) {
                if (!g2yT) {
                    g2yT = true;
                    elevator.goToPosition(18);
                    arm.setTargetPos(650);
                    armServo.setPosition(0.5);
                    flipServo.setPosition(1);
                }
            } else g2yT = false;

            if (gamepad2.b) {
                if (!g2bT) {
                    g2bT = true;
                    elevator.goToPosition(10);
                    arm.setTargetPos(650);
                    armServo.setPosition(0.5);
                    flipServo.setPosition(1);
                }
            } else g2bT = false;

            if (gamepad2.a) {
                if (!g2aT) {
                    g2aT = true;
                    elevator.goToPosition(0);
                    arm.setTargetPos(650);
                    armServo.setPosition(0.5);
                    flipServo.setPosition(1);
                }
            } else g2aT = false;

            if (gamepad2.right_stick_y > 0.5) arm.setTargetPos(arm.getTargetPos() + ARM_ADJ_RATE);
            else if (gamepad2.right_stick_y < -0.5) arm.setTargetPos(arm.getTargetPos() - ARM_ADJ_RATE);

            if (gamepad1.right_bumper && armServo.getPosition() > 0.49 && armServo.getPosition() < 0.51) {
                armServo.setPosition(0.7);
            } else if (!gamepad1.right_bumper && armServo.getPosition() > 0.69) {
                armServo.setPosition(0.5);
            }

            if (gamepad2.x) {
                if (!g2xT) {
                    g2xT = true;
                    elevator.goToPosition(0);
                    arm.setTargetPos(650);
                    armServo.setPosition(0.1);
                    flipServo.setPosition(0.35);
                }
            } else g2xT = false;

            if (gamepad1.left_bumper) {
                if (!g1lbT) {
                    g1lbT = true;
                    gripped = !gripped;
                    if (gripped) gripper.close();
                    else gripper.open();
                }
            } else g1lbT = false;


            elevator.periodic();
            arm.periodic();

            double heading = (lf.getCurrentPosition() - re.getCurrentPosition()) / (DriveConstants.TRACK_DIAMETER * DriveConstants.TICKS_PER_INCH);
            if (gamepad2.dpad_right && gamepad2.dpad_down) headOff = heading;
            drive(heading - headOff);


            double currentPos = lift1.getCurrentPosition() / ElevatorConstants.TICKS_PER_INCH;
            double currentVel = lift1.getVelocity() / ElevatorConstants.TICKS_PER_INCH;

            telemetry.addData("target", target);
            telemetry.addData("actual pos", currentPos);
            telemetry.addData("actual vel", currentVel);

            double armPos = armMotor.getCurrentPosition();

            telemetry.addData("arm pos", armPos);
            telemetry.addData("arm target", arm.getTargetPos());
            telemetry.addData("servo", armServo.getPosition());
            telemetry.addData("heading", heading);
            telemetry.addData("rfhead", rf.getCurrentPosition() / DriveConstants.TICKS_PER_INCH);
            telemetry.update();

        }
    }

    private void drive(double heading) {
        /* Drive Stuff */
        double x = -gamepad1.left_stick_x, y = gamepad1.left_stick_y, rx = -gamepad1.right_stick_x;

        double theta = Math.atan2(y, x) - Math.PI/4 - heading;
        double power = Math.hypot(x, y);
        if (gamepad1.right_trigger > 0.08 && gamepad1.right_trigger < 0.5) power *= .75;
        else if (gamepad1.right_trigger > 0.5) power *= (1 - 0.5*gamepad1.right_trigger);
        rx *= (1- 0.55*gamepad1.right_trigger);
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double maxDispPower = Math.max(Math.abs(cos), Math.abs(sin));

        double leftFront, rightFront, leftRear, rightRear;
        leftFront = power * cos / maxDispPower + rx;
        rightFront = power * sin / maxDispPower - rx;
        leftRear = power * sin / maxDispPower + rx;
        rightRear = power * cos / maxDispPower - rx;

        if (power + Math.abs(rx) > 1) {
            leftFront /= power + Math.abs(rx);
            rightFront /= power + Math.abs(rx);
            leftRear /= power + Math.abs(rx);
            rightRear /= power + Math.abs(rx);
        }

        lf.setPower(leftFront);
        rf.setPower(rightFront);
        lr.setPower(leftRear);
        rr.setPower(rightRear);
    }
}
