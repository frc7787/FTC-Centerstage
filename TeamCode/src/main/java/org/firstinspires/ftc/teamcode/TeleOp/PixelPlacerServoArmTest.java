package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Auxiliaries;

@TeleOp(name = "Test - Pixel Placer Servo")
public class PixelPlacerServoArmTest extends OpMode {

    @Override public void init() {
        Auxiliaries.init(hardwareMap);
    }

    @Override public void loop() {
       if (gamepad1.cross) {
           Auxiliaries.retractPixelPlacerLeft();
       } else if (gamepad1.square) {
           Auxiliaries.placePixelOnBackdropLeft();
       } else if (gamepad1.circle) {
           Auxiliaries.placePixelOnSpikeStripLeft();
       }

       Auxiliaries.debug(telemetry);

       telemetry.update();
    }
}
