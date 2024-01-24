package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.Hanger;

@TeleOp(name = "Test - Hanger", group = "Test")
//@Disabled
public class HangerTest extends OpMode {
    Hanger hanger;

    Gamepad currentGamepadOne, prevGamepadOne, currentGamepadTwo, prevGamepadTwo;

    @Override public void init() {
        currentGamepadOne = new Gamepad();
        prevGamepadOne    = new Gamepad();
        currentGamepadTwo = new Gamepad();
        prevGamepadTwo    = new Gamepad();

        hanger = new Hanger(hardwareMap);

        hanger.init();
    }

    @Override public void loop() {
        prevGamepadOne.copy(currentGamepadOne);
        currentGamepadOne.copy(gamepad1);

        prevGamepadTwo.copy(currentGamepadTwo);
        currentGamepadTwo.copy(gamepad2);

        hanger.debug(telemetry);

        telemetry.addLine("Press Left Bumper to release the hanging mechanism, and right bumper to return to zero position");

        if (currentGamepadOne.left_bumper && !prevGamepadOne.left_bumper || currentGamepadTwo.left_bumper && !prevGamepadTwo.left_bumper) {
            hanger.release();
        } else if (currentGamepadOne.right_bumper && !prevGamepadOne.right_bumper || currentGamepadTwo.right_bumper && !prevGamepadTwo.right_bumper) {
            hanger.init();
        }

        telemetry.update();
    }
}
