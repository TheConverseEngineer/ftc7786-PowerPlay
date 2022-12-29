package org.firstinspires.ftc.teamcode.command.triggers;

import org.firstinspires.ftc.teamcode.command.Trigger;

import java.util.function.Supplier;

public abstract class InlineTrigger extends Trigger {

    private final Supplier<Boolean> condition;

    public InlineTrigger(Supplier<Boolean> condition) {
        this.condition = condition;
    }

    @Override
    public void updateInput() {
        if (condition.get()) {
            command();
        }
    }

    /** Code inside this function will be run when the given condition is true */
    public abstract void command();
}