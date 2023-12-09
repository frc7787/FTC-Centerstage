package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.Launcher;

@TeleOp(name = "Test - Launcher", group = "Test")
@Disabled
public class LauncherTest extends OpMode {

    Launcher launcher;

    Gamepad currentGamepad, prevGamepad;

    @Override public void init() {
        currentGamepad = new Gamepad();
        prevGamepad    = new Gamepad();

        launcher = new Launcher(hardwareMap);
        launcher.init();
    }

    @Override public void loop() {
        prevGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad1);

        if (currentGamepad.left_bumper && !prevGamepad.left_bumper) {
            launcher.release();
        }
    }
}
