package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class PIDFController {
    public DoubleSupplier kP, kI, kD, kStatic, kVPositive, kVNegative, maxI, maxP, maxPower;

    private double integral, lastError, lastTime, setpoint;
    private final ElapsedTime timer;

    /** Create a new PIDF Controller
     *
     * @param kP        The proportional tuning term
     * @param kI        The integral tuning term
     * @param kD        The derivative tuning term
     * @param kStatic   The static velocity
     * @param kVPositive    The velocity tuning term when velocity is positive
     * @param kVNegative    The velocity tuning term when velocity is negative
     * @param maxI      The maximum integral contribution (including kI)
     * @param maxP      The maximum proportional contribution (including kP)
     * @param maxPower  The maximum output power of the motor (after constants)
     * @param setpoint  The current target position
     */
    public PIDFController(
            DoubleSupplier kP,
            DoubleSupplier kI,
            DoubleSupplier kD,
            DoubleSupplier kStatic,
            DoubleSupplier kVPositive,
            DoubleSupplier kVNegative,
            DoubleSupplier maxI,
            DoubleSupplier maxP,
            DoubleSupplier maxPower,
            double setpoint
    ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kStatic = kStatic;
        this.kVPositive = kVPositive;
        this.kVNegative = kVNegative;
        this.maxI = maxI;
        this.maxP = maxP;
        this.maxPower = maxPower;
        this.setpoint = setpoint;
        this.integral = 0;
        this.lastError = 0;
        this.timer = new ElapsedTime();
        this.lastTime = 0;
    }

    /** Activate the controller (start timer) */
    public void start() {
        this.timer.reset();
        this.lastTime = 0;
    }

    /** Set a new target position */
    public void setSetpoint(double newSetpoint) {
        this.setpoint = newSetpoint;
        this.integral = 0;
    }

    /** Outputs a motor power given the current position */
    public double calculate(double position, double velocity) {
        double error = this.setpoint - position;
        double deltaTime = timer.seconds() - lastTime;
        double deltaError = error - lastError;
        lastTime += deltaTime;
        if (error * lastError <= 0) integral = 0;
        else integral += error*deltaTime;
        this.lastError = error;

        return MathUtils.clamp(
                this.kStatic.getAsDouble()
                        + ((velocity >= 0) ? this.kVPositive : this.kVNegative).getAsDouble() * velocity
                        + MathUtils.clamp(this.kP.getAsDouble()*error, -this.maxP.getAsDouble(), this.maxP.getAsDouble())
                        + MathUtils.clamp(this.kI.getAsDouble()*integral, -this.maxI.getAsDouble(), this.maxI.getAsDouble())
                        + this.kD.getAsDouble()*((deltaError)/deltaTime),
                -this.maxPower.getAsDouble(), this.maxPower.getAsDouble());
    }
}
