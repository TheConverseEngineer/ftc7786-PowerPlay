package org.firstinspires.ftc.teamcode.subsystems.gripper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.Subsystem;

import static org.firstinspires.ftc.teamcode.subsystems.gripper.GripperConstants.*;

public class Gripper implements Subsystem {
    Servo leftGripServo, rightGripServo;

    public Gripper(HardwareMap map){
        //Initialize hardware
        leftGripServo = map.get(Servo.class, leftGripServoName);
        rightGripServo = map.get(Servo.class, rightGripServoName);
        //Set servos to default position
        leftGripServo.setPosition(leftGripDefaultPos);
        rightGripServo.setPosition(rightGripDefaultPos);
    }

    @Override
    public void periodic() {

    }

    //Methods to open and close the claw
    public void close(){
        leftGripServo.setPosition(leftGripClosePos);
        rightGripServo.setPosition(rightGripClosePos);
    }
    public void open(){
        leftGripServo.setPosition(leftGripOpenPos);
        rightGripServo.setPosition(rightGripOpenPos);
    }

    //Manual offset values
    //both are lumped into one method simplify control as its really all you would need to do
    //If i screwed this up and its going in the wrong direction then just flip the signs in all of these methods
    //or negate the gripOffsetVal in gripperConstants
    public void offsetOpen(){
        leftGripServo.setPosition(leftGripServo.getPosition()+gripServoOffsetVal);
        rightGripServo.setPosition(rightGripServo.getPosition()-gripServoOffsetVal);
    }

    public void offsetClose(){
        leftGripServo.setPosition(leftGripServo.getPosition()-gripServoOffsetVal);
        rightGripServo.setPosition(rightGripServo.getPosition()+gripServoOffsetVal);
    }


}
