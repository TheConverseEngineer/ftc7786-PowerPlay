package org.firstinspires.ftc.teamcode.profile;

import org.firstinspires.ftc.teamcode.susbsystems.ArmConstants;

/**
 * Copied from FTCLIB/WPILIB
 */
public class TrapezoidalProfile {
    // The direction of the profile, either 1 for forwards or -1 for inverted
    private final int directionMultiplier;

    private final State initialState;
    private final State goalState;

    private final double endAccelTime;
    private final double endFullSpeedTime;
    private final double endDeccelTime;

    /** Small data class representing a state */
    public static class State {
        public double position;
        public double velocity;

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }

    /** Construct a TrapezoidProfile given a target position, current position, and current velocity */
    public TrapezoidalProfile(double targetPosition, double currentPosition, double currentVelocity) {
        directionMultiplier = (currentPosition > targetPosition) ? -1 : 1;
        initialState = direct(new State(currentPosition, currentVelocity));
        goalState = direct(new State(targetPosition, 0.0));

        initialState.velocity = Math.min(ArmConstants.ELEVATOR_MAX_V, initialState.velocity);


        // Deal with a non-zero initial velocity
        double cutoffBegin = initialState.velocity / ArmConstants.ELEVATOR_MAX_A;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * ArmConstants.ELEVATOR_MAX_A / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double fullTrapezoidDist = cutoffDistBegin + (goalState.position - initialState.position);
        double accelerationTime = ArmConstants.ELEVATOR_MAX_V / ArmConstants.ELEVATOR_MAX_A;

        double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * ArmConstants.ELEVATOR_MAX_A;

        // Handle a degenerate profile
        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / ArmConstants.ELEVATOR_MAX_A);
            fullSpeedDist = 0;
        }

        endAccelTime = accelerationTime - cutoffBegin;
        endFullSpeedTime = endAccelTime + fullSpeedDist / ArmConstants.ELEVATOR_MAX_A;
        endDeccelTime = endFullSpeedTime + accelerationTime;
    }

    /**
     * Calculate the correct position and velocity for the profile at a time t
     * where the beginning of the profile was at time t = 0.
     *
     * @param t The time since the beginning of the profile.
     */
    public State calculate(double t) {
        State result = new State(initialState.position, initialState.velocity);

        if (t < endAccelTime) {
            result.velocity += t * ArmConstants.ELEVATOR_MAX_A;
            result.position += (initialState.velocity + t * ArmConstants.ELEVATOR_MAX_A / 2.0) * t;
        } else if (t < endFullSpeedTime) {
            result.velocity = ArmConstants.ELEVATOR_MAX_V;
            result.position += (initialState.velocity + endAccelTime * ArmConstants.ELEVATOR_MAX_A
                    / 2.0) * endAccelTime + ArmConstants.ELEVATOR_MAX_V * (t - endAccelTime);
        } else if (t <= endDeccelTime) {
            result.velocity = goalState.velocity + (endDeccelTime - t) * ArmConstants.ELEVATOR_MAX_A;
            double timeLeft = endDeccelTime - t;
            result.position = goalState.position - (goalState.velocity + timeLeft
                    * ArmConstants.ELEVATOR_MAX_A / 2.0) * timeLeft;
        } else {
            result = goalState;
        }

        return direct(result);
    }
    
    public boolean hasStoppedMoving(double t) {
        return t >= endDeccelTime;
    }

    // Flip the sign of the velocity and position if the profile is inverted
    private State direct(State in) {
        return new State(
                in.position * directionMultiplier,
                in.velocity * directionMultiplier
        );
    }
}