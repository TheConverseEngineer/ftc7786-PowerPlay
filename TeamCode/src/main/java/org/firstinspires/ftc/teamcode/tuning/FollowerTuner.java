package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AutonomousDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.TrajectorySequence;
import org.firstinspires.ftc.teamcode.virtual.VirtualDcMotorEx;
import org.firstinspires.ftc.teamcode.virtual.VirtualVoltageSensor;

@TeleOp(name = "Follower Tuner", group = "tuning")
public class FollowerTuner extends LinearOpMode {
    public static final double DISTANCE = 80; // Will move 80 inches FORWARD AND BACKWARDS
    DcMotorEx lf, rf, lr, rr;
    VoltageSensor voltage;
    AutonomousDrive drive;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();

        lf = hardwareMap.get(DcMotorEx.class, "leftfrontdrive");
        rf = hardwareMap.get(DcMotorEx.class, "rightfrontdrive");
        lr = hardwareMap.get(DcMotorEx.class, "leftreardrive");
        rr = hardwareMap.get(DcMotorEx.class, "rightreardrive");
        voltage = hardwareMap.voltageSensor.iterator().next();

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Virtual hardware
            lf = VirtualDcMotorEx.createGB312Motor();
            rf = VirtualDcMotorEx.createGB312Motor();
            lr = VirtualDcMotorEx.createGB312Motor();
            rr = VirtualDcMotorEx.createGB312Motor();
            voltage = new VirtualVoltageSensor();
        */

        drive = new AutonomousDrive(voltage, lf, rf, lr, rr, new Pose2d(0, 0, 0));

        TrajectorySequence forward = drive.trajectoryBuilder(new Pose2d(0, 0, 0), Math.PI/2)
                        .splineToSplineHeading(new Pose2d(0, DISTANCE, 0), Math.PI/2).build();

        TrajectorySequence reverse = drive.trajectoryBuilder(new Pose2d(0, DISTANCE, 0), -Math.PI/2)
                        .splineToSplineHeading(new Pose2d(0, 0, 0), -Math.PI/2).build();

        boolean goingForward = true;

        telemetry.addData("initialized", "true");
        telemetry.update();

        waitForStart();

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setNewFollowTrajectory(forward);

        while (opModeIsActive() && !isStopRequested()) {
            if (!drive.isBusy()) {
                if (goingForward) drive.setNewFollowTrajectory(reverse);
                else drive.setNewFollowTrajectory(forward);
                goingForward = !goingForward;
            }

            drive.periodic();
            TelemetryPacket packet = new TelemetryPacket();
            drive.simPeriodic(packet);

            // Add velocity data
            double forwardVel = (lf.getVelocity() + rf.getVelocity()) / (2 * DriveConstants.TICKS_PER_INCH);
            packet.put("forward velocity", forwardVel);

            dashboard.sendTelemetryPacket(packet);

        }


    }
}
