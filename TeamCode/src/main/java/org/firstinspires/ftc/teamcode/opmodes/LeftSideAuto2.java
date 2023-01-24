package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.command.prefabs.InstantCommand;
import org.firstinspires.ftc.teamcode.command.prefabs.ParallelCommand;
import org.firstinspires.ftc.teamcode.command.prefabs.SequentialCommand;
import org.firstinspires.ftc.teamcode.command.prefabs.WaitCommand;
import org.firstinspires.ftc.teamcode.command.prefabs.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.drive.AutonomousDrive;
import org.firstinspires.ftc.teamcode.drive.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.gripper.Gripper;
import org.firstinspires.ftc.teamcode.virtual.VirtualDcMotorEx;
import org.firstinspires.ftc.teamcode.virtual.VirtualServo;
import org.firstinspires.ftc.teamcode.virtual.VirtualVoltageSensor;

@Autonomous(name = "left side")
public class LeftSideAuto extends CommandOpMode {
    AutonomousDrive drive;
    Arm arm;
    Gripper gripper;
    Elevator elevator;

    DcMotor lf, rf, lr, rr, armMotor, lift1, lift2, re;
    Servo armServo, flipServo, gripperLeft, gripperRight;

    VoltageSensor voltageSensor;

    TrajectorySequence approach1, pickupCone, approach2;

    @Override
    public void init(CommandScheduler master) {
        /* START VIRTUAL HARDWARE STUFF */
        /*
        lf = VirtualDcMotorEx.createGB312Motor();
        rf = VirtualDcMotorEx.createGB312Motor();
        lr = VirtualDcMotorEx.createGB312Motor();
        rr = VirtualDcMotorEx.createGB312Motor();


        voltageSensor = new VirtualVoltageSensor();

         */
        /* END VIRTUAL HARDWARE STUFF */

        lf = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rf = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        lr = hardwareMap.get(DcMotor.class, "leftreardrive");
        rr = hardwareMap.get(DcMotor.class, "rightreardrive");
        re = hardwareMap.get(DcMotor.class, "rightencoder");

        lift1 = VirtualDcMotorEx.createGB435Motor();
        lift2 = VirtualDcMotorEx.createGB435Motor();
        armMotor = VirtualDcMotorEx.createGB435Motor();

        armServo = new VirtualServo("arm servo", 0);
        flipServo = new VirtualServo("flip servo", 1);
        gripperLeft = new VirtualServo("left", 2);
        gripperRight = new VirtualServo("right", 3);

        /*lift1 = hardwareMap.get(DcMotor.class, "liftdrive1");
        lift2 = hardwareMap.get(DcMotor.class, "liftdrive2");
        armMotor = hardwareMap.get(DcMotor.class, "armmotor");

        armServo = hardwareMap.get(ServoImplEx.class, "armservo");
        flipServo = hardwareMap.get(ServoImplEx.class, "flipservo");
        armServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        flipServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        gripperLeft = hardwareMap.get(ServoImplEx.class, "gripperleft");
        gripperRight = hardwareMap.get(ServoImplEx.class, "gripperright");
        gripperLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gripperRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        */
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
        drive = new AutonomousDrive(voltageSensor, lf, rf, lr, rr, re, new Pose2d(-37, -68, 1.5707963267948966));

        approach1 = drive.trajectoryBuilder(new Pose2d(-37, -68, Math.PI/2), 1.5707963267948966)
                .splineToLinearHeading(new Pose2d(-33, -7, -0.7400196028455958 + Math.PI/2), Math.PI/2 - 0.27925268031909267)
                .build();

        pickupCone = drive.trajectoryBuilder(new Pose2d(-33, -7, -0.7400196028455958 + Math.PI/2), -2.5830872929516078)
                .splineToLinearHeading(new Pose2d(-64, -12, -1.5707963267948966 + Math.PI/2), Math.PI)
                .build();

        approach2 = drive.trajectoryBuilder(new Pose2d(-64, -12, -1.5707963267948966+Math.PI/2), 0)
                .splineToLinearHeading(new Pose2d(-33, -7, -0.7400196028455958+Math.PI/2), -2.5830872929516078 + Math.PI)
                .build();


        master.registerSubsystem(arm, gripper, elevator, drive);
        master.setFTCDashboardStatus(true);
        master.ScheduleCommand(new SequentialCommand(
                new ParallelCommand( // First move to place the first cone
                        drive.followTrajectoryCommand(approach1),
                        new ArmToHighCommand()
                ),
                new InstantCommand(gripper::open),
                new WaitCommand(0.25),
                getCycleCommand(8),
                getCycleCommand(4),
                getCycleCommand(4),
                getCycleCommand(2),
                getCycleCommand(0)
        ));
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

    private Command getCycleCommand(double stackHeight) {
        return new SequentialCommand(
                new ParallelCommand(
                        drive.followTrajectoryCommand(pickupCone),
                        new ArmToVariableLowCommand(stackHeight),
                        new InstantCommand(arm::flipSide)
                ),
                new InstantCommand(gripper::close),
                new WaitCommand(0.25),
                SequentialCommand.runAsync(new ArmToHighCommand()),
                new WaitUntilCommand(() -> elevator.getPosition() > 16),
                new ParallelCommand(
                        drive.followTrajectoryCommand(approach2),
                        new InstantCommand(arm::flipSide)
                ),
                new InstantCommand(gripper::open),
                new WaitCommand(0.25)
        );
    }

    private class ArmToHighCommand implements Command {
        @Override
        public void init() {
            elevator.goToPosition(17);
            arm.setTargetPos(500);
            // Maybe add a armServo thing here too
        }

        @Override public void loop() { }
        @Override public void end() { }

        @Override
        public boolean isComplete() {
            return Math.abs(elevator.getPosition() - 17) <= 0.25;
        }
    }

    private class ArmToVariableLowCommand implements Command {
        private final double target;

        public ArmToVariableLowCommand(double target) {
            this.target = target;
        }

        @Override
        public void init() {
            elevator.goToPosition(target);
            arm.setTargetPos(0);
            // Maybe add a armServo thing here too
        }

        @Override public void loop() { }
        @Override public void end() { }

        @Override
        public boolean isComplete() {
            return Math.abs(elevator.getPosition() - target) <= 0.25;
        }
    }

    @Override
    public void loop(CommandScheduler master) {
        // This method should be commented out or deleted when not using virtual hardware
    }
}
