package org.firstinspires.ftc.teamcode.virtual;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VirtualVoltageSensor implements VoltageSensor {
    @Override
    public double getVoltage() {
        return 12d;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Voltage Sensor";
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
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}