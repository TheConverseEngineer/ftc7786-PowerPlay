package org.firstinspires.ftc.teamcode.archive.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.archive.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.subsystems.gripper.Gripper;

@Config
@Disabled
@TeleOp(name = "Standard TeleOp (g1++)", group = "primary")
public class StandardTeleOp2 extends LinearOpMode {

    public static double armSpeed = 0.5;

    DcMotor lf, rf, lr, rr, re;
    DcMotor lift1, lift2;
    DcMotor armMotor;

    ServoImplEx armServo, flipServo;
    ServoImplEx gripperRight, gripperLeft;

    Elevator elevator;
    Arm arm;
    Gripper gripper;

    public double elevatorPos = 0;
    public double armPos = 0;

    private boolean armClosed = false;

    private boolean g1ltoggle = false;
    private boolean g2rb = false, g2a = false, g2b = false, g2x = false, g2y = false;

    private double headingOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        lf = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rf = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        lr = hardwareMap.get(DcMotor.class, "leftreardrive");
        rr = hardwareMap.get(DcMotor.class, "rightreardrive");
        re = hardwareMap.get(DcMotor.class, "rightencoder");

        lift1 = hardwareMap.get(DcMotor.class, "liftdrive1");
        lift2 = hardwareMap.get(DcMotor.class, "liftdrive2");

        armMotor = hardwareMap.get(DcMotor.class, "armmotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armServo = hardwareMap.get(ServoImplEx.class, "armservo");
        flipServo = hardwareMap.get(ServoImplEx.class, "flipservo");
        armServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        flipServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        gripperLeft = hardwareMap.get(ServoImplEx.class, "gripperleft");
        gripperRight = hardwareMap.get(ServoImplEx.class, "gripperright");
        gripperLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gripperRight.setPwmRange(new PwmControl.PwmRange(500, 2500));


        /* START VIRTUAL HARDWARE STUFF */
        /*
        lf = VirtualDcMotorEx.createGB312Motor();
        rf = VirtualDcMotorEx.createGB312Motor();
        lr = VirtualDcMotorEx.createGB312Motor();
        rr = VirtualDcMotorEx.createGB312Motor();
        lift1 = VirtualDcMotorEx.createGB435Motor();
        lift2 = VirtualDcMotorEx.createGB435Motor();
        armMotor = VirtualDcMotorEx.createGB435Motor();

        armServo = new VirtualServo("arm servo", 0);
        flipServo = new VirtualServo("flip servo", 1);
        gripperLeft = new VirtualServo("left", 2);
        gripperRight = new VirtualServo("right", 3);
        */
        /* END VIRTUAL HARDWARE STUFF */

        telemetry = FtcDashboard.getInstance().getTelemetry();

        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        re.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        re.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator = new Elevator(lift1, lift2, 0);
        arm = new Arm(armMotor, armServo, flipServo, 0);
        gripper = new Gripper(gripperLeft, gripperRight);

        waitForStart();

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        re.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !isStopRequested()) {
            double heading = (lf.getCurrentPosition() - re.getCurrentPosition()) / (DriveConstants.TRACK_DIAMETER * DriveConstants.TICKS_PER_INCH);

            if (gamepad1.y && !g2y) {
                elevatorPos = 18.5;
                armPos = 500;
                g2y = true;
                elevator.goToPosition(elevatorPos);
                arm.setTargetPos(armPos);
            } else if (!gamepad1.y) g2y = false;

            if (gamepad1.b && !g2b) {
                elevatorPos = 7;
                armPos = 600;
                g2b = true;
                elevator.goToPosition(elevatorPos);
                arm.setTargetPos(armPos);
            } else if (!gamepad1.b) g2b = false;

            if (gamepad1.a && !g2a) {
                elevatorPos = 0;
                armPos = 500;
                g2a = true;
                elevator.goToPosition(elevatorPos);
                arm.setTargetPos(armPos);
            } else if (!gamepad1.a) g2a = false;

            if (gamepad1.x && !g2x) {
                elevatorPos = 0;
                armPos = 0;
                g2x = true;
                elevator.goToPosition(elevatorPos);
                arm.setTargetPos(armPos);
            } else if (!gamepad1.x) g2x = false;

            if (gamepad1.left_bumper && !g1ltoggle) {
                armClosed = !armClosed;
                g1ltoggle = true;
                if (armClosed) gripper.close();
                else gripper.open();
            } else if (!gamepad1.left_bumper) g1ltoggle = false;

            if (gamepad2.right_bumper && !g2rb) {
                arm.flipSide();
                g2rb = true;
            } else if (!gamepad2.right_bumper) g2rb = false;

            if (gamepad2.dpad_up) {
                armPos += armSpeed;
                arm.setTargetPos(armPos);
            }
            if (gamepad2.dpad_down) {
                armPos -= armSpeed;
                arm.setTargetPos(armPos);
            }

            if (gamepad2.a && gamepad2.b) headingOffset = heading;

            drive(heading - headingOffset);

            elevator.periodic();
            arm.periodic();
            gripper.periodic();

            telemetry.addData("elevator pos", lift1.getCurrentPosition() / ElevatorConstants.TICKS_PER_INCH);
            telemetry.addData("arm pos", armMotor.getCurrentPosition());

            telemetry.addData("elevator target", elevatorPos);
            telemetry.addData("arm target", arm.getTargetPos());

            telemetry.addData("heading", heading);
            telemetry.addData("lateral", rf.getCurrentPosition());

            telemetry.update();
        }
    }

    private void drive(double heading) {
        /* Drive Stuff */
        double x = -gamepad1.left_stick_x, y = gamepad1.left_stick_y, rx = -gamepad1.right_stick_x;

        double theta = Math.atan2(y, x) - Math.PI/4 - heading;
        double power = Math.hypot(x, y) * (1 - 0.65*gamepad1.right_trigger);
        rx *= (1- 0.65*gamepad1.right_trigger);
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
