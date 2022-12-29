package org.firstinspires.ftc.teamcode.command.prefabs;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandScheduler;

/** Runs multiple commands in order. */
public class SequentialCommand implements Command {
    private final Command[] commands;
    private int current;

    public SequentialCommand(Command... commands) {
        this.commands = commands;
        assert commands.length >= 1; // Please don't give empty command groups
        current = 0;
    }


    @Override
    public void init() {
        commands[0].init();
    }

    @Override
    public void loop() {
        if (commands[current].isComplete()) {
            commands[current].end();
            current++;
            if (current >= commands.length) return;
            commands[current].init();
        } else commands[current].loop();
    }

    @Override
    public void end() {
        if (current <= commands.length) commands[current].end();
    }

    @Override
    public boolean isComplete() {
        return current >= commands.length;
    }

    public static Command runAsync(Command command) {
        return new InstantCommand(() -> CommandScheduler.getInstance().ScheduleCommand(command));
    }
}