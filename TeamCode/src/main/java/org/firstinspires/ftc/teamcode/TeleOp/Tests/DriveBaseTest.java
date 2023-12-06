package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;

@TeleOp(name = "Test - Drive Base", group = "Test")
@Disabled
public class DriveBaseTest extends OpMode {

    private DriveBase driveBase;

    @Override public void init() {
        driveBase = new DriveBase(hardwareMap);
        driveBase.init();
    }

    @Override public void loop() {
        driveBase.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );
    }
}
