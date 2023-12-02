package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GetRobotProperties;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;

@TeleOp(name = "Test - Drive Base", group = "Test")
@Config
public class DriveBaseTest extends OpMode {

    private DriveBase driveBase;

    public static double STRAFE_OFFSET = GetRobotProperties.readDouble("STRAFE_OFFSET");

    @Override public void init() {
        driveBase = new DriveBase(hardwareMap);
        driveBase.init();
    }

    @Override public void loop() {
        driveBase.driveTest(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                STRAFE_OFFSET
        );

    }
}
