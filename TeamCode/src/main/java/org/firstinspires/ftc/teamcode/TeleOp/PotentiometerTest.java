package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Test - Potentiometer", group = "Test")
@Disabled
public class PotentiometerTest extends LinearOpMode {

    AnalogInput analogSensor;

    @Override public void runOpMode() {
        analogSensor = hardwareMap.analogInput.get("WormPotentiometer");

        waitForStart();

        while (opModeIsActive()) {
            double sensorValue = analogSensor.getVoltage();

            telemetry.addData("Voltage: ", sensorValue);

            telemetry.update();

            sleep(100);
        }
    }
}
