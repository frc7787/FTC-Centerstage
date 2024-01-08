package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_HIGH;
import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_LOW;
import static org.firstinspires.ftc.teamcode.Properties.STRAFE_OFFSET;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Test - Intake with drive base", group = "Test")
public class DriveBaseTest extends OpMode {
    DriveBase drive;
    Intake intake;

    @Override public void init() {
        intake    = new Intake(hardwareMap);
        drive     = new DriveBase(hardwareMap);

        drive.init();
        intake.init();
    }

    @Override public void loop() {
        intake.outtake(gamepad1.left_trigger);

        drive.driveManual(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }
}
