package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.command.prefabs.SequentialCommand;
import org.firstinspires.ftc.teamcode.drive.AutonomousDrive;
import org.firstinspires.ftc.teamcode.drive.TrajectorySequence;
import org.firstinspires.ftc.teamcode.virtual.VirtualDcMotorEx;
import org.firstinspires.ftc.teamcode.virtual.VirtualVoltageSensor;

@TeleOp(name = "sequence test", group = "default")
public class SequenceTest extends CommandOpMode {
    AutonomousDrive drive;

    @Override
    public void init(CommandScheduler master) {
        drive = new AutonomousDrive(new VirtualVoltageSensor(),
                VirtualDcMotorEx.createGB312Motor(),
                VirtualDcMotorEx.createGB312Motor(),
                VirtualDcMotorEx.createGB312Motor(),
                VirtualDcMotorEx.createGB312Motor(), new Pose2d(62.39, -45.82, Math.toRadians(180)));
        master.setFTCDashboardStatus(true);


        TrajectorySequence approach = drive.trajectoryBuilder(new Pose2d(68, -37, -3.141592653589793), 3.141592653589793)
                .splineToSplineHeading(new Pose2d(11, -22, 1.5707963267948966), 1.5707963267948966)
                .splineToSplineHeading(new Pose2d(18, -6, 0.7853981633974483), 0.7853981633974483)
                .build();



        master.registerSubsystem(drive);

    }
}
