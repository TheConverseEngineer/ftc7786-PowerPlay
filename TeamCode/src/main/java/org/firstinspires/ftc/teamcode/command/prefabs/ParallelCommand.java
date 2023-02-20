package org.firstinspires.ftc.teamcode.command.prefabs;

import org.firstinspires.ftc.teamcode.command.Command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ParallelCommand implements Command {

    private final ArrayList<Command> commands;

    public ParallelCommand(Command... commands) {
        this.commands = new ArrayList<>(Arrays.asList(commands));
    }

    @Override
    public void init() {
        for (Command i : commands) i.init();
    }

    @Override
    public void loop() {
        int i = 0;
        while (i < commands.size()) {
            if (commands.get(i).isComplete()) commands.remove(i).end();
            else {
                commands.get(i).loop();
                i++;
            }
        }
    }

    @Override
    public void end() {
        for (Command i : commands) i.end();
    }

    @Override
    public boolean isComplete() {
        return commands.size() == 0;
    }
}