package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.profile.TrapezoidalProfile;

@Config
@TeleOp(name = "Profile Test", group = "default")
public class ProfileTest extends LinearOpMode {

    public static double targetPos = 0;
    private double lastPos = 0;
    int count = 0;
    TrapezoidalProfile profile;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = FtcDashboard.getInstance().getTelemetry();

        telemetry.addData("status", "ready");
        telemetry.update();

        waitForStart();

        profile = new TrapezoidalProfile(0.0, 0.0, 0.0);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && !isStopRequested()) {
            TrapezoidalProfile.State current = profile.calculate(timer.seconds());
            if (Math.abs(lastPos - targetPos) > 0.0001) {
                lastPos = targetPos;
                profile = new TrapezoidalProfile(targetPos, current.position, current.velocity);
                timer.reset();
                count++;
            }

            telemetry.addData("v", current.velocity);
            telemetry.addData("s", current.position);
            telemetry.addData("time", timer.seconds());
            telemetry.addData("count", count);
            telemetry.update();
        }

    }
}
