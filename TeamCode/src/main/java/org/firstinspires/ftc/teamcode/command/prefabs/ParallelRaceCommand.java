package org.firstinspires.ftc.teamcode.command.prefabs;

import org.firstinspires.ftc.teamcode.command.Command;

public class ParallelRaceCommand implements Command {

    private final Command[] commands;
    private boolean complete;

    public ParallelRaceCommand(Command... commands) {
        this.commands = commands;
        this.complete = false;
    }

    @Override
    public void init() {
        for (Command i : commands) i.init();
    }

    @Override
    public void loop() {
        for (Command i : commands) {
            i.loop();
            complete = i.isComplete() || complete;
        }
    }

    @Override
    public void end() {
        for (Command i : commands) i.end();
    }

    @Override
    public boolean isComplete() {
        return complete;
    }
}