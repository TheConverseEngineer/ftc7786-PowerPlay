package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Stack;

public final class CommandScheduler {

    private static final CommandScheduler instance = new CommandScheduler();

    Stack<Command> queuedCommands;
    List<Command> runningCommands;
    List<Subsystem> subsystems;
    List<Trigger> triggers;
    boolean dashboardEnabled;


    private CommandScheduler() {
        runningCommands = new ArrayList<>();
        queuedCommands = new Stack<>();
        subsystems = new ArrayList<>();
        triggers = new ArrayList<>();
        dashboardEnabled = false;
    }

    public void reset() {
        runningCommands.clear();
        queuedCommands.clear();
        subsystems.clear();
        triggers.clear();
        dashboardEnabled = false;
    }

    public void setFTCDashboardStatus(boolean enabled) {
        dashboardEnabled = enabled;
    }

    public static CommandScheduler getInstance() {
        return instance;
    }

    public void ScheduleCommand(Command... cmds) {
        for (Command i : cmds) queuedCommands.push(i);
    }

    public void registerSubsystem(Subsystem... sys) {
        subsystems.addAll(Arrays.asList(sys));
    }

    public void registerTrigger(Trigger... trigger) {
        triggers.addAll(Arrays.asList(trigger));
    }

    public void run() {
        for (Subsystem i : subsystems) i.earlyPeriodic();

        for (Trigger i : triggers) i.updateInput();

        int i = 0;
        while (i < runningCommands.size()) {
            if (runningCommands.get(i).isComplete()) {
                runningCommands.remove(i).end();
            } else {
                runningCommands.get(i).loop();
                i++;
            }
        }
        while (!queuedCommands.empty()) {
            queuedCommands.peek().init();
            runningCommands.add(queuedCommands.pop());
        }

        for (Subsystem subsystem : subsystems) subsystem.periodic();

        if (this.dashboardEnabled) {
            TelemetryPacket packet = new TelemetryPacket();
            for (Subsystem subsystem: subsystems) subsystem.simPeriodic(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}