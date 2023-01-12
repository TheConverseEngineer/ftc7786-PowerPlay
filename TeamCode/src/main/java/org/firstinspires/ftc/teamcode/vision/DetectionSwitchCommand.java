package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.command.Command;

/** Calls one of 3 commands, depending on the inputted DetectionStatus */
public class DetectionSwitchCommand implements Command {

    DetectionStatus status;
    Command left, center, right;

    public DetectionSwitchCommand(DetectionStatus status, Command left, Command center, Command right) {
        this.status = status;
        this.left = left;
        this.center = center;
        this.right = right;
    }

    @Override
    public void init() {
        status.lock();
        switch (status.getPos()) {
            case LEFT: left.init();
            case CENTER: center.init();
            case RIGHT: right.init();
        }
    }

    @Override
    public void loop() {
        switch (status.getPos()) {
            case LEFT: left.loop();
            case CENTER: center.loop();
            case RIGHT: right.loop();
        }
    }

    @Override
    public void end() {
        switch (status.getPos()) {
            case LEFT: left.end();
            case CENTER: center.end();
            case RIGHT: right.end();
        }
    }

    @Override
    public boolean isComplete() {
        switch (status.getPos()) {
            case LEFT: return left.isComplete();
            case CENTER: return center.isComplete();
            case RIGHT: return right.isComplete();
            default: return true; // Will never be called
        }
    }
}
