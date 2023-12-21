package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Test - Intake with drive base", group = "Test")
public class DriveBaseTest extends OpMode {
    DriveBase driveBase;
    Intake intake;

    @Override public void init() {
        driveBase = new DriveBase(hardwareMap);
        intake    = new Intake(hardwareMap);

        driveBase.init();
        intake.init();
    }

    @Override public void loop() {
        if (gamepad1.left_bumper) {
            intake.intake();
        } else {
            intake.stop();
        }

        driveBase.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );
    }
}
