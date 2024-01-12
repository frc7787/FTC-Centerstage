package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "Turn Test", group = "Roadrunner")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = Math.PI; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        drive.init();

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(ANGLE);
    }
}
