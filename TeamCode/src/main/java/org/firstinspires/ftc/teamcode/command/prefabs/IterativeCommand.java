package org.firstinspires.ftc.teamcode.command.prefabs;

import org.firstinspires.ftc.teamcode.command.Command;

import java.util.function.Supplier;

public class IterativeCommand implements Command {
    private final Runnable command;
    private final Supplier<Boolean> isComplete;

    public IterativeCommand(Runnable command, Supplier<Boolean> isComplete) {
        this.command = command;
        this.isComplete = isComplete;
    }

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        command.run();
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isComplete() {
        return isComplete.get();
    }
}