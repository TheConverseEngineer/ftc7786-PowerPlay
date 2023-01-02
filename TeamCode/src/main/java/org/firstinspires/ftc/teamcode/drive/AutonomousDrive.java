package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.utils.DashboardUtils;
import org.firstinspires.ftc.teamcode.utils.MutablePose2d;

public class AutonomousDrive implements Subsystem {

    private final DcMotor leftFront, rightFront, leftRear, rightRear;
    private final VoltageSensor voltageSensor;
    private final MutablePose2d pose;

    private double oldLeftEncoderVal, oldRightEncoderVal, oldLateralEncoderVal;

    private final NanoClock timer;
    private double timerOffset;
    private double voltage = 12;

    @Nullable
    private TrajectorySequence currentSequence;

    /** Constructor for AutonomousDrive class.
     *      Odo wiring:
     *          left odo pod : left front drive
     *          right odo pod : right rear drive
     *          lateral odo pod: right front drive
     *
     * @param leftFront     The left front dc motor
     * @param rightFront    The right front dc motor
     * @param leftRear      The left rear dc motor
     * @param rightRear     The right rear dc motor
     * @param startPose     The robot's starting position
     */
    public AutonomousDrive(VoltageSensor voltageSensor, DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear, Pose2d startPose) {
        this.voltageSensor = voltageSensor;
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
        this.pose = new MutablePose2d(startPose.getX(), startPose.getY(), startPose.getHeading());

        this.timer = NanoClock.system();
        timerOffset = timer.seconds();
    }

    /** Returns true if this drivetrain is currently following a trajectory */
    public boolean isBusy() {
        return (timer.seconds() - timerOffset) < (currentSequence != null ? currentSequence.getDuration() : 0);
    }

    /** Returns a command, which, when scheduled, will cause the robot to follow the given trajectory sequence.
     * This command will block sequential execution until the robot has reached its final destination
     *
     * @param sequence  The TrajectorySequence to follow
     * @return     A command, which can be scheduled with the CommandScheduler
     *
     * NOTE: This method does NOT actually trigger the follower
     */
    public Command followTrajectoryCommand(TrajectorySequence sequence) {
        return new Command() {
            @Override
            public void init() {
                setNewFollowTrajectory(sequence);
            }

            @Override public void loop() { }
            @Override public void end() { }

            @Override
            public boolean isComplete() {
                return !isBusy();
            }
        };
    }

    /** Returns a command, which, when scheduled, will cause the robot to follow the given trajectory sequence.
     * This command will execute instantly, and will not block sequential execution
     *
     * @param sequence  The TrajectorySequence to follow
     * @return     A command, which can be scheduled with the CommandScheduler
     *
     * NOTE: This method does NOT actually trigger the follower
     */
    public Command followTrajectoryAsyncCommand(TrajectorySequence sequence) {
        return new Command() {
            @Override
            public void init() {
                setNewFollowTrajectory(sequence);
            }

            @Override public void loop() { }
            @Override public void end() { }

            @Override
            public boolean isComplete() {
                return true;
            }
        };
    }

    /** This method is public for specific internal purposes
     * DO NOT USE THIS METHOD
     *
     * use followTrajectoryCommand instead
     */
    public void setNewFollowTrajectory(TrajectorySequence sequence) {
        timerOffset = timer.seconds();
        currentSequence = sequence;
    }

    @Override
    public void periodic() {
        voltage = voltageSensor.getVoltage();

        // Update Odometry
        double dl = getLeftEncoderDelta() / DriveConstants.TICKS_PER_INCH;
        double dr = getRightEncoderDelta() / DriveConstants.TICKS_PER_INCH;
        double dLat = getLateralEncoderDelta() / DriveConstants.TICKS_PER_INCH;
        pose.integrateLocalDisp((dl + dr)/2.0,
                                dLat - (dr - dl)*(DriveConstants.LATERAL_RADIUS / DriveConstants.TRACK_DIAMETER),
                            (dr - dl) / (DriveConstants.TRACK_DIAMETER));

        if (currentSequence != null) {
            // Continues to "follow" trajectory even if it is complete
            double time = timer.seconds() - timerOffset;
            Pose2d target = currentSequence.get(time);
            Pose2d velocity = currentSequence.velocity(time);
            Pose2d acceleration = currentSequence.acceleration(time);

            // Apply proportional controller and run movement code
            driveFieldCentric(
                    velocity.getX() + DriveConstants.kPLateral * (target.getX() - pose.x),
                    velocity.getY() + DriveConstants.kPLateral * (target.getY() - pose.y),
                    velocity.getHeading() + DriveConstants.kPRotational * (target.getHeading() - pose.theta),
                    acceleration.getX(),
                    acceleration.getY(),
                    acceleration.getHeading()
            );
        } else {
            // If no trajectory exists, then sit stationary
            setMotorPowers(0, 0, 0, 0);
        }
    }

    private double getLeftEncoderDelta() {
        double newPos = leftFront.getCurrentPosition();
        double delta = newPos - oldLeftEncoderVal;
        oldLeftEncoderVal = newPos;
        return delta;
    }

    private double getRightEncoderDelta() {
        double newPos = rightRear.getCurrentPosition();
        double delta = newPos - oldRightEncoderVal;
        oldRightEncoderVal = newPos;
        return delta;
    }

    private double getLateralEncoderDelta() {
        double newPos = rightFront.getCurrentPosition();
        double delta = newPos - oldLateralEncoderVal;
        oldLateralEncoderVal = newPos;
        return delta;
    }

    @Override
    public void simPeriodic(TelemetryPacket packet) {
        if (currentSequence != null) {
            double time = timer.seconds();
            Pose2d target = currentSequence.get(time - timerOffset);

            // Draw target pose
            DashboardUtils.drawRobot(target, "green", packet.fieldOverlay());
            packet.put("target y velocity", currentSequence.velocity(time - timerOffset).getY());

            // Log error
            packet.put("y error", target.getY() - pose.y);
            packet.put("x error", target.getX() - pose.x);
            packet.put("heading error", target.getHeading() - pose.theta);
        }

        // Draw robot
        DashboardUtils.drawRobot(pose, "blue", packet.fieldOverlay());
    }

    /** Returns a trajectory builder instance */
    public TrajectorySequence.Builder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectorySequence.Builder(startPose, startHeading);
    }

    /** Sets the raw motor powers */
    private void setMotorPowers(double lf, double rf, double lr, double rr) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }

    /** Computes inverse kinematics for the drivetrain given a velocity and angular velocity
     * in the global frame
     *
     * Input is fed into here from the P-controllers and trajectory velocity output
     * The Feedforward controller is used here to convert these wheel velocities into raw motor
     * power
     */
    private void driveFieldCentric(double vx, double vy, double vw, double ax, double ay, double aw) {
        // First compute velocity and acceleration
        double[] vels = mecanumIK(vx, vy, vw);
        double[] accels = mecanumIK(ax, ay, aw);

        // Now pass through feedforward
        setMotorPowers(
                feedforward(vels[0], accels[0]),
                feedforward(vels[1], accels[1]),
                feedforward(vels[2], accels[2]),
                feedforward(vels[3], accels[3])
        );
    }

    /** Computes mecanum inverse kinematics given global-frame input
     *
     * Can be used with both velocity and acceleration
     * @return  an array of four motor values, arranged lf, rf, lr, rr
     */
    private double[] mecanumIK(double x, double y, double w) {
        // First rotate the input so it correlates to the local frame instead of the global one
        double x2 = Math.cos(pose.theta)*x + Math.sin(pose.theta)*y;
        double y2 = Math.cos(pose.theta)*y - Math.sin(pose.theta)*x;

        // Now actually calculate and return the IK values
        return new double[]{
                x2 - y2 * DriveConstants.LATERAL_MULTIPLIER - w * DriveConstants.TRACK_DIAMETER,
                x2 + y2 * DriveConstants.LATERAL_MULTIPLIER + w * DriveConstants.TRACK_DIAMETER,
                x2 + y2 * DriveConstants.LATERAL_MULTIPLIER - w * DriveConstants.TRACK_DIAMETER,
                x2 - y2 * DriveConstants.LATERAL_MULTIPLIER + w * DriveConstants.TRACK_DIAMETER
        };
    }

    /** Applies the feedforward controller. kS, kV, and kA can b found in DriveConstants */
    private double feedforward(double v, double a) {
        // Sacrificing readability for execution speed, hence the ternary operators
        return (DriveConstants.kS * ((v > -0.0001 && v < 0.0001) ? 0 : (v > 0) ? 1 : -1) +
                DriveConstants.kV * v + DriveConstants.kA * a) / voltage;
    }


}
