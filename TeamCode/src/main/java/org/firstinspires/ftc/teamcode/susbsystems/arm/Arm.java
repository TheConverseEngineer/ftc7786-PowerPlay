package org.firstinspires.ftc.teamcode.susbsystems.arm;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.susbsystems.arm.ArmConstants.*;

import org.firstinspires.ftc.teamcode.command.Subsystem;

public class Arm implements Subsystem {
    //Decalring hardware
    DcMotorEx armMotor;
    Servo armServo;

    ElapsedTime timer;

    //target positions
    private double armMotorTargetPos = ARM_DEFAULT_POS;
    private double flipServoTargetPos = ARM_DEFAULT_POS;
    private double fourbarServoTargetPos = ARM_DEFAULT_POS;

    //PID helper values
    private double lastI = 0;
    private double armPos = 0;
    private double lastError = 0;
    private double lastTime;

    public Arm(HardwareMap map){
        armMotor = map.get(DcMotorEx.class, "armMotor");
        armServo = map.get(Servo.class, "armServo");

        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void periodic() {
        //Update position values
        armPos = armMotor.getCurrentPosition();
        loopArmPID();
    }

    public void simPeriodic() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("-------ARM-------");
        packet.addLine("-----ARM END-----");
        Subsystem.super.simPeriodic(packet);
    }

    public void loopArmPID(){
        double error = armMotorTargetPos - armPos;
        double deltaTime = timer.time()-lastTime;
        double PVal = error;
        //If error changes sign set I to zero
        if(error*lastError<=0){
            lastI = 0;
        }

        double IVal;
        if(Math.abs(lastI)<I_CAP) {
            IVal = error*deltaTime + lastI;
        }
        else{
            IVal = lastI;
        }

        double DVal = error-lastError*deltaTime;

        double output = (PVal*P_PID) + (IVal*I_PID) + (DVal*D_PID);
        lastError = error;
        lastTime = timer.time();
        armMotor.setPower(output);
    }

    //Offset Util Methods
    public void offsetMotorUp(){
        setArmMotorTargetPos(getArmMotorTargetPos()+ARM_MOTOR_OFFSET);
    }
    public void offsetMotorDown(){
        setArmMotorTargetPos(getArmMotorTargetPos()-ARM_MOTOR_OFFSET);
    }
    public void offsetFourbarServoUp(){
        setFourbarServoTargetPos(getFourbarServoTargetPos()+FOURBAR_SERVO_OFFSET);
    }
    public void offsetFourbarServoDown(){
        setFourbarServoTargetPos(getFourbarServoTargetPos()-FOURBAR_SERVO_OFFSET);
    }
    public void offsetFlipServoUp(){
        setFourbarServoTargetPos(getFourbarServoTargetPos()+FLIP_SERVO_OFFSET);
    }
    public void offsetFlipServoDown(){
        setFourbarServoTargetPos(getFourbarServoTargetPos()-FLIP_SERVO_OFFSET);
    }

    //All them accessors
    public double getArmMotorTargetPos() {
        return armMotorTargetPos;
    }

    public void setArmMotorTargetPos(double armMotorTargetPos) {
        //Set PID I value to zero when the motor target changes
        lastI = 0;
        this.armMotorTargetPos = armMotorTargetPos;
    }

    public double getFlipServoTargetPos() {
        return flipServoTargetPos;
    }

    public void setFlipServoTargetPos(double flipServoTargetPos) {
        this.flipServoTargetPos = flipServoTargetPos;
    }

    public double getFourbarServoTargetPos() {
        return fourbarServoTargetPos;
    }

    public void setFourbarServoTargetPos(double fourbarServoTargetPos) {
        this.fourbarServoTargetPos = fourbarServoTargetPos;
    }
}