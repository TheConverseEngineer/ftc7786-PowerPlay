package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public interface Subsystem {

    /** Runs on every loop iteration before commands are executed */
    default void earlyPeriodic() {

    }

    /** Runs on every loop iteration after commands are executed */
    void periodic();

    /** Runs every loop iteration after periodic if FTCDashboard is enabled */
    default void simPeriodic(TelemetryPacket packet) {

    }
}