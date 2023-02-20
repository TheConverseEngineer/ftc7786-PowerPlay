package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.utils.DashboardUtils;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.MutablePose2d;

@Config
public class AutoDriveV3 implements Subsystem {

    public static double TICKS_PER_INCH = 887.540704;
    public static double TRACK_DIAMETER = 11.731521505;
    public static double LATERAL_RADIUS = 5;

    public static double kP, kStatic, kPRotational, kStaticRotational;
    public static double maxSpeed, maxTurnSpeed;

    private final DcMotor lf, rf, lr, rr, re;
    private int lastLf = 0, lastRf = 0, lastRe = 0;

    MutablePose2d pose;
    Pose2d target;

    public AutoDriveV3(DcMotor lf, DcMotor rf, DcMotor lr, DcMotor rr, DcMotor re, Pose2d initialPos) {
        this.lf = lf;
        this.rf = rf;
        this.lr = lr;
        this.rr = rr;
        this.re = re;
        this.target = initialPos;
        pose = new MutablePose2d(initialPos.getX(), initialPos.getY(), initialPos.getHeading());
    }

    public void setTarget(Pose2d target) {
        this.target = target;
    }

    private double getLeftEncoderDelta() {
        int delta = lf.getCurrentPosition() - lastLf;
        lastLf += delta;
        return delta;
    }

    private double getRightEncoderDelta() {
        int delta = re.getCurrentPosition() - lastRe;
        lastRe += delta;
        return -delta;
    }

    private double getRearEncoderDelta() {
        int delta = rf.getCurrentPosition() - lastRf;
        lastRf += delta;
        return delta;
    }

    @Override
    public void periodic() {
        double dl = getLeftEncoderDelta() / TICKS_PER_INCH;
        double dr = getRightEncoderDelta() / TICKS_PER_INCH;
        double dLat = getRearEncoderDelta() / TICKS_PER_INCH;

        pose.integrateLocalDisp(
                dLat - (dr - dl)*(LATERAL_RADIUS / TRACK_DIAMETER),
                (dl + dr)/2.0,
                (dr - dl) / (TRACK_DIAMETER)
        );

        double xError = target.getX() - pose.x;
        double yError = target.getY() - pose.y;
        double dist = Math.hypot(xError, yError);
        double angleError = MathUtils.wrap(target.getHeading() - pose.theta);

        drive(
                Math.atan2(yError, xError),
                MathUtils.clamp(
                        (dist > 0.0001)?kStatic:0 + dist*kP,
                       0, maxSpeed
                ),
                MathUtils.clamp(
                        (Math.abs(angleError) > 0.0001)?kStaticRotational:0 + angleError*kPRotational,
                        -maxTurnSpeed, maxTurnSpeed
                )
        );
    }

    @Override
    public void simPeriodic(TelemetryPacket packet) {
        DashboardUtils.drawRobot(pose, "blue", packet.fieldOverlay());
        DashboardUtils.drawRobot(target, "green", packet.fieldOverlay());
        double xError = target.getX() - pose.x;
        double yError = target.getY() - pose.y;
        double dist = Math.hypot(xError, yError);
        packet.put("error", dist);
        packet.put("angle", Math.toDegrees(Math.atan2(yError, xError)));
    }

    /** Drive in a specified direction at a specified speed
     *
     * @param direction     The direction to drive in
     * @param power         The speed at which to drive
     * @param rx            The speed at which to turn
     */
    private void drive(double direction, double power, double rx) {
        double theta = direction - pose.theta - Math.PI/4;
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double maxDispPower = Math.max(Math.abs(cos), Math.abs(sin));

        double leftFront, rightFront, leftRear, rightRear;
        leftFront = power * cos / maxDispPower + rx;
        rightFront = power * sin / maxDispPower - rx;
        leftRear = power * sin / maxDispPower + rx;
        rightRear = power * cos / maxDispPower - rx;

        if (power + Math.abs(rx) > 1) {
            leftFront /= power + Math.abs(rx);
            rightFront /= power + Math.abs(rx);
            leftRear /= power + Math.abs(rx);
            rightRear /= power + Math.abs(rx);
        }

        lf.setPower(leftFront);
        rf.setPower(rightFront);
        lr.setPower(leftRear);
        rr.setPower(rightRear);
    }
}
