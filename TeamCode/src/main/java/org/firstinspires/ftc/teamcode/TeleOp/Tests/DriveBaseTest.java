package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Test - Intake with drive base", group = "Test")
@Disabled
public class DriveBaseTest extends OpMode {
    DriveBase drive;

    @Override public void init() {
        drive = new DriveBase(hardwareMap);
        drive.init();
    }

    @Override public void loop() {
        telemetry.addLine("Use the Joysticks to drive the robot around");

        drive.driveManualRobotCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }
}
