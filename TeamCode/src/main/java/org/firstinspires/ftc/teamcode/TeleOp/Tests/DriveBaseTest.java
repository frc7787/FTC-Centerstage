package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Test - Intake with drive base", group = "Test")
public class DriveBaseTest extends OpMode {
    MecanumDriveBase drive;
    Intake intake;

    @Override public void init() {
        intake    = new Intake(hardwareMap);

        intake.init();
    }

    @Override public void loop() {
        intake.outtake(gamepad1.left_trigger);

        drive.driveManual(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );
    }
}
