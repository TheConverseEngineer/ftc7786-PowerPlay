package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.virtual.VirtualDcMotorEx;
import org.firstinspires.ftc.teamcode.virtual.VirtualServo;

@Disabled
@TeleOp(name = "Arm Test", group = "default")
public class ArmTest extends LinearOpMode {
    DcMotor armMotor;
    Servo armServo;
    Servo flipServo;
    DcMotor elevatorMotor1;
    DcMotor elevatorMotor2;

    Arm arm;
    Elevator elevator;

    public static boolean flip = false;
    public static double target = -1;
    public static double elevatorTarget = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = VirtualDcMotorEx.createGB435Motor();
        armServo = new VirtualServo("arm servo", 0);
        flipServo = new VirtualServo("flip servo", 1);

        elevatorMotor1 = VirtualDcMotorEx.createGB435Motor();
        elevatorMotor2 = VirtualDcMotorEx.createGB435Motor();

        arm = new Arm(armMotor, armServo, flipServo, 0);
        elevator = new Elevator(elevatorMotor1, elevatorMotor2, 0);

        telemetry = FtcDashboard.getInstance().getTelemetry();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (flip) {
                flip = false;
                arm.flipSide();
            }
            if (target >= 0) {
                arm.setTargetPos(target);
                target = -1;
            }

            if (elevatorTarget >= 0) {
                elevator.goToPosition(elevatorTarget);
                elevatorTarget = -1;
            }

            arm.periodic();
            elevator.periodic();

            telemetry.addData("arm pos", armMotor.getCurrentPosition());
            telemetry.addData("arm target", arm.getTargetPos());

            telemetry.addData("flip servo", flipServo.getPosition());
            telemetry.addData("arm servo", armServo.getPosition());

            telemetry.addData("elevator pos", (elevatorMotor1.getCurrentPosition() + elevatorMotor2.getCurrentPosition())/2);
            telemetry.update();
        }
    }
}
