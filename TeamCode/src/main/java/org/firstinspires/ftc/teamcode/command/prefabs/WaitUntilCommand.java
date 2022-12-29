package org.firstinspires.ftc.teamcode.command.prefabs;

import org.firstinspires.ftc.teamcode.command.Command;

import java.util.function.Supplier;

public class WaitUntilCommand implements Command {

    private final Supplier<Boolean> condition;

    public WaitUntilCommand(Supplier<Boolean> condition) {
        this.condition = condition;
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void end() {

    }

    @Override
    public boolean isComplete() {
        return condition.get();
    }
}