package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.vision.CameraSubsystem;
import org.firstinspires.ftc.teamcode.vision.DetectAprilTagCommand;
import org.firstinspires.ftc.teamcode.vision.DetectionStatus;

@Disabled
@TeleOp(name = "vision test", group = "default")
public class VisionTest extends CommandOpMode {
    CameraSubsystem camera;
    DetectionStatus status;

    ElapsedTime timer;

    @Override
    public void init(CommandScheduler master) {
        camera = new CameraSubsystem(true);
        status = new DetectionStatus();
        master.registerSubsystem(camera);
        master.ScheduleCommand(new DetectAprilTagCommand(camera, status, 5));

        telemetry = FtcDashboard.getInstance().getTelemetry();

        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void loop(CommandScheduler master) {
        telemetry.addData("detected", status.getPos().toString());
    }
}
