package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.utils.MathUtils;

import java.util.Arrays;

/** Wrapper class for roadrunner's trajectory class
 * Designed to simplify integration
 * Also comes with its own builder
 */
public class TrajectorySequence {
    private final Trajectory trajectory;
    private final double duration;

    protected TrajectorySequence(Trajectory trajectory) {
        this.trajectory = trajectory;
        this.duration = trajectory.duration();
    }

    /** Returns the current target position in the trajectory, automatically considering bounds */
    public Pose2d get(double t) {
        return trajectory.get(MathUtils.clamp(t, 0, duration));
    }

    /** Returns the current target velocity in the trajectory, automatically considering bounds */
    public Pose2d velocity(double t) {
        if (t < 0 || t > duration) return new Pose2d(0, 0, 0);
        else return trajectory.velocity(t);
    }

    /** Returns the current target acceleration in the trajectory, automatically considering bounds */
    public Pose2d acceleration(double t) {
        if (t < 0 || t > duration) return new Pose2d(0, 0, 0);
        else return trajectory.acceleration(t);
    }

    /** Returns the total duration of this profile */
    public double getDuration() {
        return duration;
    }

    /** Wrapper for Trajectory builder */
    public static class Builder {
        private final TrajectoryBuilder internalBuilder;

        public Builder(Pose2d startPose, double splineHeading) {
            TrajectoryVelocityConstraint velocityConstraint = new MinVelocityConstraint(
                    Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANGULAR_VELOCITY),
                            new MecanumVelocityConstraint(DriveConstants.MAX_VELOCITY, DriveConstants.TRACK_DIAMETER)));
            TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(DriveConstants.MAX_ACCELERATION);

            internalBuilder = new TrajectoryBuilder(startPose, splineHeading,
                    velocityConstraint, accelerationConstraint, DriveConstants.PROFILE_RESOLUTION);
        }

        public Builder splineToSplineHeading(Pose2d pose, double splineHeading) {
            internalBuilder.splineToSplineHeading(pose, splineHeading);
            return this;
        }

        public Builder splineToLinearHeading(Pose2d pose, double splineHeading) {
            internalBuilder.splineToLinearHeading(pose, splineHeading);
            return this;
        }

        public TrajectorySequence build() {
            return new TrajectorySequence(internalBuilder.build());
        }
    }
}
