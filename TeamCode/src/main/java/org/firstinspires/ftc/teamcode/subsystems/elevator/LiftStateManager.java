package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftStateManager {

    private int directionMultiplier;

    private State initialState;
    private State goalState;

    private double endAccelTime;
    private double endFullSpeedTime;
    private double endDeccelTime;

    private final ElapsedTime timer;
    private double resetTime;

    public LiftStateManager(double startPosition) {
        endAccelTime = -1;
        endFullSpeedTime = -1;
        endDeccelTime = -1;
        timer = new ElapsedTime();
        timer.reset();
        resetTime = 0;
        initialState = new State(startPosition, 0);
        goalState = new State(startPosition, 0);
    }

    public void setNewTarget(double newPos) {
        resetTime = timer.seconds();
        State current = this.calculate(resetTime);
        this.setNewTarget(newPos, current.position, current.velocity);
    }

    public double getMotorPower(double currentPosition) {
        State target = this.calculate(timer.seconds() - resetTime);
        return ElevatorConstants.ELEVATOR_kStatic +
                ElevatorConstants.ELEVATOR_kV * target.velocity +
                ElevatorConstants.ELEVATOR_P * (target.position - currentPosition);
    }

    /** Small utility class representing a state */
    public static class State {
        public double position;
        public double velocity;

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

    }

    /** Inverts a state if needed (reflexive normalization function) */
    private State direct(State in) {
        return new State(
                in.position * directionMultiplier,
                in.velocity * directionMultiplier
        );
    }

    /** Creates a new trapezoidal profile to move to the new target position */
    private void setNewTarget(double targetPosition, double currentPosition, double currentVelocity) {
        directionMultiplier = (currentPosition > targetPosition) ? -1 : 1;
        initialState = direct(new State(currentPosition, currentVelocity));
        goalState = direct(new State(targetPosition, 0.0));

        initialState.velocity = Math.min(ElevatorConstants.ELEVATOR_MAX_V, initialState.velocity);


        // Deal with a non-zero initial velocity
        double cutoffBegin = initialState.velocity / ElevatorConstants.ELEVATOR_MAX_A;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * ElevatorConstants.ELEVATOR_MAX_A / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double fullTrapezoidDist = cutoffDistBegin + (goalState.position - initialState.position);
        double accelerationTime = ElevatorConstants.ELEVATOR_MAX_V / ElevatorConstants.ELEVATOR_MAX_A;

        double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * ElevatorConstants.ELEVATOR_MAX_A;

        // Handle a degenerate profile
        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / ElevatorConstants.ELEVATOR_MAX_A);
            fullSpeedDist = 0;
        }

        endAccelTime = accelerationTime - cutoffBegin;
        endFullSpeedTime = endAccelTime + fullSpeedDist / ElevatorConstants.ELEVATOR_MAX_A;
        endDeccelTime = endFullSpeedTime + accelerationTime;
    }

    private State calculate(double t) {
        State result = new State(initialState.position, initialState.velocity);

        if (t < endAccelTime) {
            result.velocity += t * ElevatorConstants.ELEVATOR_MAX_A;
            result.position += (initialState.velocity + t * ElevatorConstants.ELEVATOR_MAX_A / 2.0) * t;
        } else if (t < endFullSpeedTime) {
            result.velocity = ElevatorConstants.ELEVATOR_MAX_V;
            result.position += (initialState.velocity + endAccelTime * ElevatorConstants.ELEVATOR_MAX_A
                    / 2.0) * endAccelTime + ElevatorConstants.ELEVATOR_MAX_V * (t - endAccelTime);
        } else if (t <= endDeccelTime) {
            result.velocity = goalState.velocity + (endDeccelTime - t) * ElevatorConstants.ELEVATOR_MAX_A;
            double timeLeft = endDeccelTime - t;
            result.position = goalState.position - (goalState.velocity + timeLeft
                    * ElevatorConstants.ELEVATOR_MAX_A / 2.0) * timeLeft;
        } else {
            result = goalState;
        }

        return direct(result);
    }


}
