package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class PIDController {
    public DoubleSupplier kP, kI, kD, maxI, maxPower;

    private double integral, lastError, lastTime, setpoint;
    private final ElapsedTime timer;

    /** Create a new PID Controller
     *
     * @param kP        The proportional tuning term
     * @param kI        The integral tuning term
     * @param kD        The derivative tuning term
     * @param maxI      The maximum integral contribution (including kI)
     * @param maxPower  The maximum output power of the motor (after constants)
     * @param setpoint  The current target position
     */
    public PIDController(DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD, DoubleSupplier maxI, DoubleSupplier maxPower, double setpoint) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxI = maxI;
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
    public double calculate(double position) {
        double error = this.setpoint - position;
        double deltaTime = timer.seconds() - lastTime;
        lastTime += deltaTime;
        if (error * lastError <= 0) integral = 0;
        else integral += error*deltaTime;
        double deltaError = error - lastError;
        this.lastError = error;

        return MathUtils.clamp(
                this.kP.getAsDouble()*error
                   + MathUtils.clamp(this.kI.getAsDouble()*integral, -this.maxI.getAsDouble(), this.maxI.getAsDouble())
                   + this.kD.getAsDouble()*(deltaError/deltaTime),
                -this.maxPower.getAsDouble(), this.maxPower.getAsDouble());
    }


}
