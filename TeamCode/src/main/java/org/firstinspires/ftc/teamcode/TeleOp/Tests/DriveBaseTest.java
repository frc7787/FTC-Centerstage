package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;

@TeleOp(name = "Test - Drive Base", group = "Test")
public final class DriveBaseTest extends OpMode {

    public DriveBase driveBase;

    @Override public void init() { driveBase = new DriveBase(this); }

    @Override public void loop() {
        driveBase.run();
        driveBase.debug();
    }
}
