package org.firstinspires.ftc.teamcode.command.prefabs;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.Command;

public class WaitCommand implements Command {
    private final ElapsedTime timer;
    private final long waitTime;

    /** Constructor for WaitCommand
     * @param waitTime  time to wait, in seconds
     */
    public WaitCommand(double waitTime) {
        timer = new ElapsedTime();
        this.waitTime = (long) (waitTime * 1000);
    }


    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void loop() {

    }

    @Override
    public void end() {

    }

    @Override
    public boolean isComplete() {
        return timer.milliseconds() > this.waitTime;
    }
}