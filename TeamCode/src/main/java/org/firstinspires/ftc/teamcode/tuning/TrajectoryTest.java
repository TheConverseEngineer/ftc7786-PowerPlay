package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp(name = "rr test", group = "default")
public class TrajectoryTest extends LinearOpMode {

    Trajectory trajectory;
    TrajectoryBuilder builder;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    final double resolution = .25;
    final TrajectoryVelocityConstraint velocityConstraint = new MinVelocityConstraint(
            Arrays.asList(new AngularVelocityConstraint(Math.PI/3),
                    new MecanumVelocityConstraint(30, 13)));
    final TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(30);

    @Override
    public void runOpMode() throws InterruptedException {
        builder = new TrajectoryBuilder(new Pose2d(-35, -70, Math.PI/2), Math.PI/2, velocityConstraint, accelerationConstraint, resolution);
        Trajectory untitled0 = builder.splineTo(new Vector2d(-34.36, 0.82), Math.toRadians(6.84))
                .splineTo(new Vector2d(21.48, 2.05), Math.toRadians(-63.43))
                .splineTo(new Vector2d(24.14, -30.27), Math.toRadians(227.91))
                .splineTo(new Vector2d(10.64, -72.82), Math.toRadians(270.00))
                .build();

        dashboard.getTelemetry().addData("Duration", untitled0.duration());
        dashboard.getTelemetry().update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {
            double time = timer.seconds();
            telemetry.addData("time", time);
            if (time <= untitled0.duration()) {
                Pose2d target = untitled0.get(time);
                Pose2d vel = untitled0.velocity(time);
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().fillCircle(target.getX(), target.getY(), 10);
                packet.put("vx", vel.getX());
                packet.put("vy", vel.getY());
                packet.put("vHeading", vel.getHeading());
                dashboard.sendTelemetryPacket(packet);
            }
        }

    }
}
