package org.firstinspires.ftc.teamcode.virtual;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/** Virtual representation of a servo
 *
 * Implements all servo methods
 */
public class VirtualServo implements Servo {
    private final String name;
    private final int portNum;
    private Direction direction;
    private double position;

    public VirtualServo(String name, int portNum) {
        this.name = name;
        this.portNum = portNum;
        this.direction = Direction.FORWARD;
        this.position = 0;
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
    public void setPosition(double position) {
        this.position = position;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public void scaleRange(double min, double max) { }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "";
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