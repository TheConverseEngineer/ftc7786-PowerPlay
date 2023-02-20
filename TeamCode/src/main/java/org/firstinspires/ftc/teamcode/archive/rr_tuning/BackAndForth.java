package org.firstinspires.ftc.teamcode.archive.rr_tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.archive.drive.AutoDriveV2;
import org.firstinspires.ftc.teamcode.utils.DashboardUtils;

@Config
@Disabled
@TeleOp(name = "back and forth", group = "rr")
public class BackAndForth extends LinearOpMode {
    AutoDriveV2 drive;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double DISTANCE = 50;

    public boolean goingForward = true;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new AutoDriveV2(hardwareMap, new Pose2d(0, 0, 0));

        Trajectory forward = drive.getTrajectoryBuilder(new Pose2d(0, 0, 0), 0)
                .forward(DISTANCE).build();

        Trajectory back = drive.getTrajectoryBuilder(new Pose2d(0, 0, 0), Math.PI)
                .back(DISTANCE).build();

        waitForStart();

        drive.initiate();
        drive.followTrajectory(forward);

        while (!isStopRequested() && opModeIsActive()) {
            if (!drive.isBusy()) {
                goingForward = !goingForward;
                if (goingForward) drive.followTrajectory(forward);
                else drive.followTrajectory(back);
            }

            TelemetryPacket packet = new TelemetryPacket();
            drive.periodic();
            drive.simPeriodic(packet);
            Pose2d pose = drive.getPoseEstimate();
            DashboardUtils.drawRobot(pose, "blue", packet.fieldOverlay());
            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("theta", pose.getHeading());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
