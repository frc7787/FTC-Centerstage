package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GetRobotProperties;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Test - Intake", group = "Test")
@Config
public class IntakeTest extends OpMode {

    private Intake intake;

    public static double INTAKE_SPEED = GetRobotProperties.readDouble("INTAKE_SPEED");
    public static double OUTTAKE_SPEED = GetRobotProperties.readDouble("OUTTAKE_SPEED");

    @Override public void init() { intake = new Intake(hardwareMap); }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            intake.intakeTest(INTAKE_SPEED);
        } else if (gamepad1.right_bumper) {
            intake.outtakeTest(OUTTAKE_SPEED);
        }
    }
}
