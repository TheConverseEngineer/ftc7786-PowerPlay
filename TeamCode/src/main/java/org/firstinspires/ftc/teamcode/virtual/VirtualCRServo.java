package org.firstinspires.ftc.teamcode.virtual;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/** Virtual representation of a CRServo
 *
 * Implements all CRServo methods
 */
public class VirtualCRServo implements CRServo {
    private final String name;
    private final int portNum;
    private DcMotor.Direction direction;
    private double power;

    public VirtualCRServo(String name, int portNum) {
        this.name = name;
        this.portNum = portNum;
        this.direction = DcMotor.Direction.FORWARD;
        this.power = 0;
    }

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return portNum;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return this.direction;
    }

    @Override
    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public double getPower() {
        return this.power;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return this.name;
    }

    @Override
    public String getConnectionInfo() {
        return "";
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