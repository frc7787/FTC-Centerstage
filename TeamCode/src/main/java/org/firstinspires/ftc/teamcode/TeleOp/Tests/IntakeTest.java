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

    Gamepad currentGamepad, prevGamepad;

    @Override public void init() {
        currentGamepad = new Gamepad();
        prevGamepad    = new Gamepad();

        intake = new Intake(hardwareMap);

    }

    @Override
    public void loop() {
        prevGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad1);

       intake.debug(telemetry);

        if (currentGamepad.left_bumper && prevGamepad.right_bumper) {
            intake.intake();
        }

        telemetry.update();
    }
}
