package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.DeliveryTray;

@TeleOp(name = "Test - Delivery Tray", group = "Test")
@Disabled
public class DeliveryTrayTest extends OpMode {

    DeliveryTray tray;

    Gamepad currentGamepadOne, prevGamepadOne, currentGamepadTwo, prevGamepadTwo;

    public void init() {
        currentGamepadOne = new Gamepad();
        prevGamepadOne    = new Gamepad();
        currentGamepadTwo = new Gamepad();
        prevGamepadTwo    = new Gamepad();

        tray = new DeliveryTray(hardwareMap);

        tray.init();
    }

    public void loop() {
        telemetry.addLine("Press X to open the door, and circle to close it");
        telemetry.addLine("Press Dpad Up to move the tray up, and Dpad Down to move the tray down");

        tray.debug(telemetry);

        if (currentGamepadOne.cross && !prevGamepadOne.cross || currentGamepadTwo.cross && !prevGamepadTwo.cross) {
            tray.openDoor();
        } else if (currentGamepadOne.circle && !prevGamepadOne.circle || currentGamepadTwo.circle && !prevGamepadTwo.circle) {
            tray.closeDoor();
        }

        if (currentGamepadOne.dpad_up && !prevGamepadOne.dpad_up || currentGamepadTwo.dpad_up && !prevGamepadTwo.dpad_up) {
            tray.move_tray(1);
        } else if (currentGamepadOne.dpad_down && !prevGamepadOne.dpad_down || currentGamepadTwo.dpad_down && !prevGamepadTwo.dpad_down) {
            tray.move_tray(0);
        }


        telemetry.update();
    }
}
