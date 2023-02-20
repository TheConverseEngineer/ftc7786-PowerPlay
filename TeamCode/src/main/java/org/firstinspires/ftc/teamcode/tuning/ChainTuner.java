package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "CHAIN TUNER", group = "chain")
@Disabled
public class ChainTuner extends LinearOpMode {

    ServoImplEx servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(ServoImplEx.class, "armservo");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) servo.setPosition(0.15);
            else if (gamepad1.b) servo.setPosition(0.65);
        }
    }
}
