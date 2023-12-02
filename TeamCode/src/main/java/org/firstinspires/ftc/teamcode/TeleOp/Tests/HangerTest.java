package org.firstinspires.ftc.teamcode.TeleOp.Tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GetRobotProperties;
import org.firstinspires.ftc.teamcode.Subsytems.Hanger;

@TeleOp(name = "Test - Hanger", group = "Test")
@Config
public class HangerTest extends OpMode {

    private Hanger hanger;

    public static double leftServoPos = GetRobotProperties.readDouble("HANGER_SERVO_POSITION");
    public static double rightServoPos = GetRobotProperties.readDouble("HANGER_SERVO_POSITION");

    @Override public void init() {
        hanger = new Hanger(hardwareMap);
        hanger.init();
    }

    @Override public void loop() {
        if (gamepad1.triangle) { hanger.releaseTest(leftServoPos, rightServoPos); }
    }
}
