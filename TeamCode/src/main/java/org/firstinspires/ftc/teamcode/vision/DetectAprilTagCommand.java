package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.Command;

/** Simple command that runs the april tag detection for a specified duration and saves the output */
public class DetectAprilTagCommand implements Command {

    DetectionStatus status;
    CameraSubsystem camera;
    double detectionTime;
    ElapsedTime timer;

    public DetectAprilTagCommand(CameraSubsystem camera, DetectionStatus status, double detectionTime) {
        this.camera = camera;
        this.status = status;
        this.detectionTime = detectionTime;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        timer.reset();
        this.camera.initialize();
    }

    @Override
    public void loop() { }

    @Override
    public void end() {
        status.setPosition(ParkPosition.aprilCodeToPosition(camera.close()));
    }

    @Override
    public boolean isComplete() {
        return (timer.seconds() >= detectionTime);
    }
}
