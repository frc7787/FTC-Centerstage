package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Auxiliaries;

@TeleOp(name = "Test - Pixel Placer Servo")
@Disabled
public class PixelPlacerServoArmTest extends OpMode {

    @Override public void init() {
        Auxiliaries.init(hardwareMap);
    }

    @Override public void loop() {
       if (gamepad1.dpad_down) {
           Auxiliaries.retractPixelPlacerLeft();
       } else if (gamepad1.dpad_left) {
           Auxiliaries.placePixelOnBackdropLeft();
       } else if (gamepad1.dpad_right) {
           Auxiliaries.placePixelOnSpikeStripLeft();
       }

       if (gamepad1.cross) {
           Auxiliaries.retractPixelPlacerRight();
       } else if (gamepad1.square) {
           Auxiliaries.placePixelOnBackdropRight();
       } else if (gamepad1.circle) {
           Auxiliaries.placePixelOnSpikeStripRight();
       }


       //Auxiliaries.debug(telemetry);

       telemetry.update();
    }
}
