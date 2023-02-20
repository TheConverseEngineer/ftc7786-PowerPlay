package org.firstinspires.ftc.teamcode.archive.rr_tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.archive.drive.AutoDriveV2;
import org.firstinspires.ftc.teamcode.utils.DashboardUtils;

@TeleOp(name = "local test", group = "rr")
@Disabled
public class LocalizationTest extends LinearOpMode {
    AutoDriveV2 drive;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new AutoDriveV2(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        drive.initiate();

        while (!isStopRequested() && opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.periodic();

            TelemetryPacket packet = new TelemetryPacket();
            Pose2d pose = drive.getPoseEstimate();
            DashboardUtils.drawRobot(pose, "blue", packet.fieldOverlay());
            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("theta", pose.getHeading());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
