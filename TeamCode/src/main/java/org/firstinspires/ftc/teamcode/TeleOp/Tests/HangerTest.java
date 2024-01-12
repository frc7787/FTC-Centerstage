package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.Hanger;

@TeleOp(name = "Test - Hanger", group = "Test")
@Disabled
public class HangerTest extends OpMode {

    Hanger hanger;

    Gamepad currentGamepad, prevGamepad;

    @Override public void init() {
        currentGamepad = new Gamepad();
        prevGamepad    = new Gamepad();

        hanger = new Hanger(hardwareMap);
    }

    @Override public void loop() {
        prevGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        if (currentGamepad.triangle && !prevGamepad.triangle) {
            hanger.release();
        }
    }
}
