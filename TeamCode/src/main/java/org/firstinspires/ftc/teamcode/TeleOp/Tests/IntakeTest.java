package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends OpMode {

    // Creates a new intake object
    public static Intake intake;

    @Override public void init() { intake = new Intake(this); }

    @Override public void loop() { intake.run(); }
}
