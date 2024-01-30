package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "Test - Rev Color Sensor")
public class RevColorSensorTest extends OpMode {

    RevColorSensorV3 colorSensorV3;
    Intake intake;

    double distance;

    double distanceThresholdMM = 7;

    @Override public void init() {
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "Color Sensor");
        intake = new Intake(hardwareMap);

        intake.init();
        colorSensorV3.initialize();
    }

    @Override public void loop() {
        distance = colorSensorV3.getDistance(DistanceUnit.MM);

        if (distance <= distanceThresholdMM) {
            telemetry.addLine("Detecting Something");
        } else {
            telemetry.addLine("Not Detecting Anything");
        }

        telemetry.update();
    }
}
