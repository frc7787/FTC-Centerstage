package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "Turn Test", group = "Roadrunner")
@Disabled
public class TurnTest extends LinearOpMode {
    public static double ANGLE = Math.PI * 10.0d; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        drive.init();

        waitForStart();

        if (isStopRequested()) return;


        drive.turn(0);
        sleep(5000);
        drive.turn(ANGLE);
    }
}
