package org.firstinspires.ftc.teamcode.archive.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.utils.DashboardUtils;
import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.MutablePose2d;

import java.util.Arrays;
import java.util.List;

@Config
/* TODO:
 * odo turns when moving forward and vice versa -> fixed (reversed right power)
 * jittery movement - maybe fixed? (fixed track width and wheel base)
 */
public class AutoDriveV2 implements Subsystem {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double MAX_VEL = 20, MAX_ACCEL = 20, MAX_ANG_VEL = Math.toRadians(60), TRACK_WIDTH = 11.731521505, LATERAL_RADIUS = 5;
    public static double WHEEL_TRACK_WIDTH = 14, WHEEL_WHEEL_BASE = 14;
    public static double kV = 0.01, kA = 0, kStatic = 0;

    public static double LATERAL_MULTIPLIER = 1.1;
    public static double TICKS_PER_INCH = 887.540704;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private final TrajectoryFollower follower;

    private final DcMotorEx leftFront, leftRear, rightRear, rightFront, rightEncoderMotor;
    private final Encoder rightEncoder, leftEncoder, rearEncoder;

    private final MutablePose2d pose;
    private Pose2d currentVelocity = new Pose2d(0, 0, 0);

    private boolean trajectorySet = false;

    public AutoDriveV2(HardwareMap hardwareMap, Pose2d startPose) {
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftfrontdrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftreardrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightreardrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightfrontdrive");
        rightEncoderMotor = hardwareMap.get(DcMotorEx.class, "rightencoder");

        for (DcMotorEx motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        rightEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // All three encoders should be reversed
        rightEncoder = new Encoder(rightEncoderMotor, Encoder.Direction.FORWARD);
        leftEncoder = new Encoder(leftFront, Encoder.Direction.REVERSE);
        rearEncoder = new Encoder(rightFront, Encoder.Direction.REVERSE);

        pose = new MutablePose2d(startPose.getX(), startPose.getY(), startPose.getHeading());
    }

    public void initiate() {
        rightEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        // Update pose
        double dl = leftEncoder.getDeltaPos() / TICKS_PER_INCH;
        double dr = rightEncoder.getDeltaPos() / TICKS_PER_INCH;
        double dLat = rearEncoder.getDeltaPos() / TICKS_PER_INCH;
        pose.integrateLocalDisp(
                (dl + dr)/2.0,
                dLat + (dr - dl)*(LATERAL_RADIUS / TRACK_WIDTH),
                (dr - dl) / (TRACK_WIDTH)
        );

        double dlV = leftEncoder.getCorrectedVelocity() / TICKS_PER_INCH;
        double drV = rightEncoder.getCorrectedVelocity() / TICKS_PER_INCH;
        double dLatV = rearEncoder.getCorrectedVelocity() / TICKS_PER_INCH;
        currentVelocity = new Pose2d(
                (dlV + drV)/2.0,
                dLatV + (drV - dlV)*(LATERAL_RADIUS / TRACK_WIDTH),
                (drV - dlV) / (TRACK_WIDTH)
        );

        DriveSignal signal = (trajectorySet) ?
                follower.update(pose.asPose(), currentVelocity) :
                new DriveSignal();
        setDriveSignal(signal);
    }

    @Override
    public void simPeriodic(TelemetryPacket packet) {
        DashboardUtils.drawRobot(follower.getTrajectory().get(follower.elapsedTime()), "green", packet.fieldOverlay());
        packet.put("target v", follower.getTrajectory().velocity(follower.elapsedTime()).getY());
        packet.put("actual v", currentVelocity.getY());
    }

    public boolean isBusy() {
        return follower.isFollowing();
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
    public void setDriveSignal(DriveSignal driveSignal) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocities(
                driveSignal.getVel(),
                WHEEL_TRACK_WIDTH,
                WHEEL_WHEEL_BASE,
                LATERAL_MULTIPLIER
        );
        List<Double> accelerations = MecanumKinematics.robotToWheelAccelerations(
                driveSignal.getAccel(),
                WHEEL_TRACK_WIDTH,
                WHEEL_WHEEL_BASE,
                LATERAL_MULTIPLIER
        );
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    public void followTrajectory(Trajectory trajectory) {
        trajectorySet = true;
        follower.followTrajectory(trajectory);
    }

    public Command getFollowTrajectoryCommand(Trajectory trajectory) {
        return new Command() {
            @Override
            public void init() {
                followTrajectory(trajectory);
            }
            @Override public void loop() { }
            @Override public void end() { }
            @Override
            public boolean isComplete() {
                return isBusy();
            }
        };
    }

    public Command getFollowTrajectoryAsyncCommand(Trajectory trajectory) {
        return new Command() {
            @Override
            public void init() {
                followTrajectory(trajectory);
            }
            @Override public void loop() { }
            @Override public void end() { }
            @Override
            public boolean isComplete() {
                return true;
            }
        };
    }

    public Pose2d getPoseEstimate() {
        return pose.asPose();
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = Math.abs(drivePower.getX())
                    + Math.abs(drivePower.getY())
                    + Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    drivePower.getX(),
                    drivePower.getY(),
                    drivePower.getHeading()
            ).div(denom);
        }

        List<Double> powers = MecanumKinematics.robotToWheelVelocities(
                drivePower,
                WHEEL_TRACK_WIDTH,
                WHEEL_WHEEL_BASE,
                LATERAL_MULTIPLIER
        );
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    public TrajectoryBuilder getTrajectoryBuilder(Pose2d startPose, double splineHeading) {
        return new TrajectoryBuilder(startPose, splineHeading,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT, 0.25);
    }
}
