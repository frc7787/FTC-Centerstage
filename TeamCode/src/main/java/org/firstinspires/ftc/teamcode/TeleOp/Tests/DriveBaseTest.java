package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;

@TeleOp(name = "Test - Drive Base", group = "Test")
@Disabled
public class DriveBaseTest extends OpMode {

    private DriveBase driveBase;

    @Override public void init() { driveBase = new DriveBase(hardwareMap); }

    @Override public void loop() {
        driveBase.drive(
                gamepad2.left_stick_x,
                gamepad2.left_stick_y,
                gamepad2.right_stick_x
        );

        driveBase.debug(telemetry);
    }
}
