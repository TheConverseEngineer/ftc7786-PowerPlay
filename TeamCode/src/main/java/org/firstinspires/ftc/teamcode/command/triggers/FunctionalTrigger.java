package org.firstinspires.ftc.teamcode.command.triggers;

import org.firstinspires.ftc.teamcode.command.Trigger;

import java.util.function.Supplier;

public class FunctionalTrigger extends Trigger {

    private final Runnable command;
    private final Supplier<Boolean> condition;

    /** Constructor for FunctionalTrigger Class
     * To avoid breaking, please ensure that the inputted runnable does not use blocking loops.
     * If blocking loops are required, consider using StandardTrigger and a Command instance instead.
     */
    public FunctionalTrigger(Supplier<Boolean> condition, Runnable command) {
        this.command = command;
        this.condition = condition;
    }

    @Override
    public void updateInput() {
        if (condition.get()) {
            command.run();
        }
    }
}
