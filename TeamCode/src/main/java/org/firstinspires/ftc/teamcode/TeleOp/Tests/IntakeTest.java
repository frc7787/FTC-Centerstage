package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Test - Intake", group = "Test")
@Disabled
public class IntakeTest extends OpMode {

    private Intake intake;

    @Override public void init() { intake = new Intake(hardwareMap); }

    @Override
    public void loop() {
       intake.debug(telemetry);

        if (gamepad1.left_bumper) {
            intake.intake();
        }

        telemetry.update();
    }
}
