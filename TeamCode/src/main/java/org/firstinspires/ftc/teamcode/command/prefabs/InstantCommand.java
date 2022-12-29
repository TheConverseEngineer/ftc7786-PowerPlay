package org.firstinspires.ftc.teamcode.command.prefabs;

import org.firstinspires.ftc.teamcode.command.Command;

public class InstantCommand implements Command {
    private final Runnable command;

    public InstantCommand(Runnable command) {
        this.command = command;
    }

    @Override
    public void init() {
        command.run();
    }

    @Override
    public void loop() {
    }

    @Override
    public void end() {
    }

    @Override
    public boolean isComplete() {
        return true;
    }
}