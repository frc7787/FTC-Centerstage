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
       Auxiliaries.setPixelPlacerServoLeftTargetPosition(gamepad1.left_stick_y * -1);
       Auxiliaries.setPixelPlacerServoRightTargetPosition(gamepad1.right_stick_x * -1);

       Auxiliaries.debug(telemetry);

       telemetry.update();
    }
}
