package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

public class CameraSubsystem implements Subsystem {

    OpenCvCamera camera;
    AprilTagPipeline pipeline;

    boolean dashboardEnabled;

    public static double TAG_SIZE = .0508; // meters

    public CameraSubsystem(boolean dashboardEnabled) {
        this.dashboardEnabled = dashboardEnabled;

        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK);
        pipeline = new AprilTagPipeline(TAG_SIZE);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                if (dashboardEnabled)
                    camera.startStreaming(640,480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                    FtcDashboard.getInstance().startCameraStream(camera, 15);
            }

            @Override
            public void onError(int errorCode) { }
        });
    }

    @Override
    public void periodic() { }

    public void initialize() {
        this.pipeline.beginCounting();
    }

    public int close() {
        int answer = pipeline.getMaxFinds();
        this.camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                if (dashboardEnabled)
                    camera.stopStreaming();
                    FtcDashboard.getInstance().stopCameraStream();
            }
        });
        return answer;
    }


}
