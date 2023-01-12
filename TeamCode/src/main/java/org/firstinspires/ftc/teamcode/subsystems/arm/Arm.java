package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class Arm implements Subsystem {
    private final DcMotor armMotor;
    private final Servo armServo, flipServo;

    private final ElapsedTime timer;

    // target positions
    private double armMotorTargetPos;

    // PID helper values
    private double lastI = 0;
    private double armPos = 0;
    private double lastError = 0;
    private double lastTime;

    private boolean flipped = false;

    /** Assumes that the arm is originally facing forward
     *
     * @param armMotor      The dc motor controlling the arm
     * @param armServo      The four-bar servo
     * @param flipServo     The servo that connects the gripper to the arm
     * @param armStartPosition  The default starting position of the arm
     */
    public Arm(DcMotor armMotor, Servo armServo, Servo flipServo, double armStartPosition) {
        this.armMotor = armMotor;
        this.armServo = armServo;
        this.flipServo = flipServo;

        timer = new ElapsedTime();
        timer.reset();
        armMotorTargetPos = armStartPosition;
    }

    public void setTargetPos(double armMotorTargetPos) {
        // Set PID I value to zero when the motor target changes
        lastI = 0;
        this.armMotorTargetPos = MathUtils.clamp(armMotorTargetPos, ArmConstants.ARM_MIN_POS, ArmConstants.ARM_MAX_POS / 2);
        if (flipped)
            this.armMotorTargetPos = ArmConstants.ARM_MAX_POS - this.armMotorTargetPos;
    }

    public double getTargetPos() {
        return this.armMotorTargetPos;
    }

    /** Flips the arm around the robot */
    public void flipSide() {
        armServo.setPosition(flipped ? ArmConstants.FOUR_BAR_SERVO_FORWARD : ArmConstants.FOUR_BAR_SERVO_REVERSE);
        flipServo.setPosition(flipped ? ArmConstants.FLIP_SERVO_REVERSE : ArmConstants.FLIP_SERVO_FORWARD);
        this.setTargetPos(ArmConstants.ARM_MAX_POS - armMotorTargetPos);
        flipped = !flipped;
    }


    @Override
    public void periodic() {
        //Update position values
        armPos = armMotor.getCurrentPosition();
        if (armPos < ArmConstants.ARM_MIN_POS) armMotor.setPower(.25);
        else if (armPos > ArmConstants.ARM_MAX_POS) armMotor.setPower(-.25);
        else loopArmPID();
    }

    public void loopArmPID(){
        double error = armMotorTargetPos - armPos;
        double deltaTime = timer.seconds()-lastTime;
        //If error changes sign set I to zero
        if(error*lastError<=0) lastI = 0;

        double IVal = MathUtils.clamp(error*deltaTime + lastI, -ArmConstants.I_CAP, ArmConstants.I_CAP);
        double DVal = (error-lastError)/deltaTime;
        double output = (error*ArmConstants.P_PID) + (IVal*ArmConstants.I_PID) + (DVal*ArmConstants.D_PID);

        lastError = error;
        lastTime += deltaTime;
        armMotor.setPower(output);
    }

    /* Todo - rewrite offset messages
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
    } */
}