package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;

@TeleOp(name = "Drive Base Test", group = "Test")
public final class DriveBaseTest extends OpMode {

    public DriveBase driveBase;

    @Override
    public void init() { driveBase = new DriveBase(this); }

    @Override
    public void loop() {
        driveBase.drive();
        driveBase.debug();
    }
}
