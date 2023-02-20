package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.archive.drive.TeleOpDrive;

@Disabled
@TeleOp(name = "Drive Test", group = "default")
public class TeleOpDriveTest extends CommandOpMode {
    TeleOpDrive drive;
    DcMotor lf, rf, lr, rr;

    @Override
    public void init(CommandScheduler master) {
        lf = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rf = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        lr = hardwareMap.get(DcMotor.class, "leftreardrive");
        rr = hardwareMap.get(DcMotor.class, "rightreardrive");

        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive = new TeleOpDrive(lf, rf, lr, rr, 0);

        master.registerSubsystem(drive);
        master.setFTCDashboardStatus(true);
    }

    @Override
    public void start(CommandScheduler master) {
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(CommandScheduler master) {
        drive.gamepadControl(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
