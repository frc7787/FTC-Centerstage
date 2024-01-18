package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.DeliveryTray;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends OpMode {

    DeliveryTray tray;

    public void init() {
        tray = new DeliveryTray(hardwareMap);

        tray.init();
    }

    public void loop() {
        tray.debug(telemetry);

        if (gamepad1.cross) {
            tray.openDoor();
        } else if (gamepad1.circle) {
            tray.closeDoor();
        }

        if (gamepad1.dpad_up) {
            tray.move_tray(1);
        } else if (gamepad1.dpad_down) {
            tray.move_tray(0);
        }
    }
}
