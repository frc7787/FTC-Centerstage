package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;

@TeleOp(name = "Test - Drive Base", group = "Test")
@Disabled
public class DriveBaseTest extends OpMode {

    @Override public void init() {
        DriveBase.init(hardwareMap);
    }

    @Override public void loop() {
        double drive  = gamepad1.left_stick_y * -1.0; // Left stick y is inverted
        double strafe = gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

        DriveBase.driveManualRobotCentric(drive, strafe, turn);
    }
}
