package org.firstinspires.ftc.teamcode.virtual;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Virtual Representation of a DcMotor
 *  All DcMotorEx methods are implemented correctly (including encoder readings)
 *  An internal timer is used to calculate current position
 *
 *  Note that using deprecated methods will throw UnsupportedOperationExceptions
 *
 * @author TheConverseEngineer
 */
public class VirtualDcMotorEx implements DcMotorEx {
    private boolean motorEnabled = true;
    private final String name;
    private double currentPower;
    private Direction currentDirection;
    private RunMode runMode;
    private int targetPosition;
    private ZeroPowerBehavior zpb;
    private final int portNum;
    private int targetPositionTolerance;
    private PIDFCoefficients veloCoeffs, pCoeffs;
    private final int rpm;
    private final double cpr;
    private final ElapsedTime timer;
    private double lastReset;
    private double currentPosition;

    public VirtualDcMotorEx(String name, int portNum, int rpm, double cpr) {
        this.name = name;
        this.currentPower = 0;
        this.currentDirection = Direction.FORWARD;
        this.runMode = RunMode.RUN_WITHOUT_ENCODER;
        this.targetPosition = 0;
        this.zpb = ZeroPowerBehavior.BRAKE;
        this.portNum = portNum;
        this.targetPositionTolerance = 3;
        this.veloCoeffs = new PIDFCoefficients(0, 0, 0, 0);
        this.pCoeffs = new PIDFCoefficients(0, 0, 0, 0);
        this.rpm = rpm;
        this.cpr = cpr;
        this.timer = new ElapsedTime();
        this.lastReset = timer.milliseconds();
        this.currentPosition = 0;
    }

    /** Factory method to create a GoBilda 435rpm yellow jacket virtual motor */
    public static VirtualDcMotorEx createGB435Motor() {
        return new VirtualDcMotorEx("gb_yellow_jacket:435rpm", 0, 435, 384.5);
    }

    /** Factory method to create a GoBilda 312rpm yellow jacket virtual motor */
    public static VirtualDcMotorEx createGB312Motor() {
        return new VirtualDcMotorEx("gb_yellow_jacket:312rpm", 0, 312, 537.7);
    }

    private void update(double motorPower) {
        currentPosition += (timer.milliseconds() - lastReset) * (rpm / 60000d * cpr * this.currentPower);
        lastReset = timer.milliseconds();
        this.currentPower = motorPower;
    }

    @Override
    public void setMotorEnable() {
        motorEnabled = true;
    }

    @Override
    public void setMotorDisable() {
        motorEnabled = false;
    }

    @Override
    public boolean isMotorEnabled() {
        return motorEnabled;
    }

    @Override
    public void setVelocity(double angularRate) {
        this.update((60d*angularRate)/(rpm*cpr));
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        switch (unit){
            case DEGREES: this.update((60d*angularRate)/(rpm*360d)); break;
            case RADIANS: this.update((60d*angularRate)/(rpm*2d*Math.PI));
        }
    }

    @Override
    public double getVelocity() {
        return this.currentPower * rpm / 60d * cpr;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        switch(unit){
            case DEGREES: return this.currentPower * rpm / 60d * 360;
            case RADIANS: return this.currentPower * rpm / 60d * 2 * Math.PI;
            default: return 0;
        }
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        if (mode == RunMode.RUN_USING_ENCODER) this.veloCoeffs = pidfCoefficients;
        else if (mode == RunMode.RUN_TO_POSITION) this.pCoeffs = pidfCoefficients;
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        this.veloCoeffs.p = p;
        this.veloCoeffs.i = i;
        this.veloCoeffs.d = d;
        this.veloCoeffs.f = f;
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        this.pCoeffs.p = p;
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        throw new UnsupportedOperationException();
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        if (mode == RunMode.RUN_USING_ENCODER) return this.veloCoeffs;
        else if (mode == RunMode.RUN_TO_POSITION) return this.pCoeffs;
        return null;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        this.targetPositionTolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return this.targetPositionTolerance;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 1;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 12;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        // No body cares about this
    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return MotorConfigurationType.getUnspecifiedMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) { }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return this.portNum;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zpb = zeroPowerBehavior;
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zpb;
    }

    @Override
    public void setPowerFloat() {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean getPowerFloat() {
        return Math.abs(currentPower) < 0.02;
    }

    @Override
    public void setTargetPosition(int position) {
        this.targetPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return this.targetPosition;
    }

    @Override
    public boolean isBusy() {
        return Math.abs(this.currentPower - this.targetPosition) < this.targetPositionTolerance;
    }

    @Override
    public int getCurrentPosition() {
        return (int) this.currentPosition;
    }

    @Override
    public void setMode(RunMode mode) {
        if (mode != RunMode.STOP_AND_RESET_ENCODER) this.runMode = mode;
    }

    @Override
    public RunMode getMode() {
        return this.runMode;
    }

    @Override
    public void setDirection(Direction direction) {
        currentDirection = direction;
    }

    @Override
    public Direction getDirection() {
        return currentDirection;
    }

    @Override
    public void setPower(double power) {
        this.update(power);
    }

    @Override
    public double getPower() {
        return currentPower;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return name;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() { }

    @Override
    public void close() { }
}