package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Intake Test", group = "Test")
@Disabled
public class IntakeTest extends OpMode {

    // Creates a new intake object
    public static Intake intake;

    @Override public void init() { intake = new Intake(hardwareMap); }

    @Override public void loop() {
        if (gamepad2.left_bumper) {
            intake.intake();
        } else if (gamepad2.right_bumper) {
            intake.hold();
        } else if (gamepad2.right_trigger > 0.9) {
            intake.outtake();
        }
        intake.debug(telemetry);
    }
}
