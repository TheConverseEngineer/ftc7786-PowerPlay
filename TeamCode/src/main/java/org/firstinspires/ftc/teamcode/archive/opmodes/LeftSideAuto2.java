package org.firstinspires.ftc.teamcode.archive.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.command.prefabs.InstantCommand;
import org.firstinspires.ftc.teamcode.command.prefabs.SequentialCommand;
import org.firstinspires.ftc.teamcode.archive.drive.AutonomousDrive;
import org.firstinspires.ftc.teamcode.archive.drive.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.gripper.Gripper;
import org.firstinspires.ftc.teamcode.vision.CameraSubsystem;
import org.firstinspires.ftc.teamcode.vision.DetectAprilTagCommand;
import org.firstinspires.ftc.teamcode.vision.DetectionStatus;
import org.firstinspires.ftc.teamcode.vision.DetectionSwitchCommand;

@Config
@Autonomous(name = "left side (reliable)")
public class LeftSideAuto2 extends CommandOpMode {

    public static double XD = -20;
    public static double YD = 54;
    public static double head = -4*Math.PI / 25;
    AutonomousDrive drive;
    Arm arm;
    Gripper gripper;
    Elevator elevator;
    CameraSubsystem camera;

    DcMotor lf, rf, lr, rr, armMotor, lift1, lift2, re;
    ServoImplEx armServo, flipServo, gripperLeft, gripperRight;

    VoltageSensor voltageSensor;


    @Override
    public void init(CommandScheduler master) {
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
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        re.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        re.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm = new Arm(armMotor, armServo, flipServo, 0);
        gripper = new Gripper(gripperLeft, gripperRight);
        elevator = new Elevator(lift1, lift2, 0);
        drive = new AutonomousDrive(voltageSensor, lf, rf, lr, rr, re, new Pose2d(0, 0, 0));

        camera = new CameraSubsystem(true);

        TrajectorySequence forward = drive.trajectoryBuilder(new Pose2d(0, 0, 0), Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, 52, 0), Math.PI/2).build();

        TrajectorySequence right = drive.trajectoryBuilder(new Pose2d(0, 52, 0), 0).
                splineToLinearHeading(new Pose2d(-23.5, 52, 0), 0).build();

        TrajectorySequence left = drive.trajectoryBuilder(new Pose2d(0, 52, 0), 0)
                .splineToLinearHeading(new Pose2d(23.5, 52, 0), 0).build();

        DetectionStatus status = new DetectionStatus();

        master.registerSubsystem(arm, gripper, elevator, drive);
        // master.setFTCDashboardStatus(true);
        master.ScheduleCommand(
                new SequentialCommand(
                    new DetectAprilTagCommand(camera, status, 5),
                    drive.followTrajectoryCommand(forward),
                    new InstantCommand(() -> {telemetry.addData("detected", status.getPos().name()); telemetry.update();}),
                    new DetectionSwitchCommand(status,
                        drive.followTrajectoryCommand(left),
                        new InstantCommand(() -> telemetry.addData("foo", "bar")),
                        drive.followTrajectoryCommand(right)
                    )
                )
        );
    }

    @Override
    public void start(CommandScheduler master) {
        gripper.close();
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        re.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void loop(CommandScheduler master) {
        // This method should be commented out or deleted when not using virtual hardware
    }
}
