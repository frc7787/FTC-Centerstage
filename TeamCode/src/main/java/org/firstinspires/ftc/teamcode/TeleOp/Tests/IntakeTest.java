package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Test - Intake", group = "Test")
@Disabled
public class IntakeTest extends OpMode {

    Intake intake;

    Gamepad currentGamepadOne, prevGamepadOne, currentGamepadTwo, prevGamepadTwo;

    boolean intakeToggle = false;

    @Override public void init() {
        currentGamepadOne = new Gamepad();
        prevGamepadOne    = new Gamepad();
        currentGamepadTwo = new Gamepad();
        prevGamepadTwo    = new Gamepad();

        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        prevGamepadOne.copy(currentGamepadOne);

        currentGamepadOne.copy(gamepad1);

       intake.debug(telemetry);

        if (currentGamepadOne.left_bumper && !prevGamepadOne.left_bumper || currentGamepadTwo.left_bumper && !prevGamepadTwo.left_bumper) {
            intakeToggle = !intakeToggle;
        }

        if (intakeToggle) {
            intake.intake();
        } else {
            intake.stop();
        }

        telemetry.update();
    }
}
