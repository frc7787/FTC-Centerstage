package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Launcher;

@TeleOp(name = "Test - Launcher", group = "Test")
@Disabled
public class LauncherTest extends OpMode {

    private Launcher launcher;

    @Override public void init() {
        launcher = new Launcher(hardwareMap);
        launcher.init();
    }

    @Override public void loop() {
        if (gamepad1.triangle) { launcher.release(); }
    }
}
