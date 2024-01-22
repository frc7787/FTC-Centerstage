package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.Launcher;

@TeleOp(name = "Test - Launcher", group = "Test")
//@Disabled
public class LauncherTest extends OpMode {
    Launcher launcher;

    Gamepad currentGamepadOne, prevGamepadOne, currentGamepadTwo, prevGamepadTwo;

    @Override public void init() {
        currentGamepadOne = new Gamepad();
        prevGamepadOne    = new Gamepad();
        currentGamepadTwo = new Gamepad();
        prevGamepadTwo    = new Gamepad();

        launcher = new Launcher(hardwareMap);
        launcher.init();
    }

    @Override public void loop() {
        prevGamepadOne.copy(currentGamepadOne);
        currentGamepadOne.copy(gamepad1);

        telemetry.addLine("Press Left Bumper to release the launcher.");
        telemetry.addLine("Press Right Bumper to zero the launcher.");

        launcher.debug(telemetry);

        if (currentGamepadOne.left_bumper && !prevGamepadOne.left_bumper || currentGamepadTwo.left_bumper && !prevGamepadTwo.left_bumper) {
            launcher.release();
        } else if (currentGamepadOne.right_bumper && !prevGamepadOne.right_bumper || currentGamepadTwo.right_bumper && !prevGamepadTwo.right_bumper) {
            launcher.zero();
        }

        telemetry.update();
    }
}
