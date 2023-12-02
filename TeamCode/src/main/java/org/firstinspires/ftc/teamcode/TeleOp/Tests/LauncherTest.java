package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GetRobotProperties;
import org.firstinspires.ftc.teamcode.Subsytems.Launcher;

@TeleOp(name = "Test - Launcher", group = "Test")
@Config
public class LauncherTest extends OpMode {

    private Launcher launcher;

    public static double LAUNCH_POSITION = GetRobotProperties.readDouble("LAUNCHER_SERVO_POSITION");

    @Override public void init() {
        launcher = new Launcher(hardwareMap);
        launcher.init();
    }

    @Override public void loop() {
        if (gamepad1.triangle) { launcher.releaseTest(LAUNCH_POSITION); }
    }
}
