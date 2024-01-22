package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;

@TeleOp(name = "Test - Arm Current", group = "Test")
public class ArmCurrentTest extends OpMode {
    Arm arm;

    Gamepad currentGamepadOne, prevGamepadOne, currentGamepadTwo, prevGamepadTwo;

    double maxElevatorCurrent = 0.0d;
    double maxWormCurrent     = 0.0d;
    double maxArmCurrent      = 0.0d;

    public void init() {
        currentGamepadOne = new Gamepad();
        prevGamepadOne    = new Gamepad();
        currentGamepadTwo = new Gamepad();
        prevGamepadTwo    = new Gamepad();

        arm = new Arm(hardwareMap);
        arm.init();
    }

    public void loop() {
        prevGamepadOne.copy(currentGamepadOne);
        currentGamepadOne.copy(gamepad1);

        prevGamepadTwo.copy(currentGamepadTwo);
        currentGamepadTwo.copy(gamepad2);

        if (arm.getElevatorCurrentAmps() > maxElevatorCurrent) {
            maxElevatorCurrent = arm.getElevatorCurrentAmps();
        }

        if (arm.getWormCurrentAmps() > maxWormCurrent) {
            maxWormCurrent = arm.getWormCurrentAmps();
        }

        if (arm.getArmCurrentAmps() > maxArmCurrent) {
            maxArmCurrent = arm.getArmCurrentAmps();
        }

        telemetry.addData("Max Worm Current", maxWormCurrent);
        telemetry.addData("Max Elevator Current", maxElevatorCurrent);
        telemetry.addData("Max Arm Current", maxArmCurrent);

        if (currentGamepadOne.cross && !prevGamepadOne.cross || currentGamepadTwo.cross && !prevGamepadTwo.cross) {
            arm.moveToPosition(3045, 2326);
        } else if (currentGamepadOne.circle && !prevGamepadOne.circle || currentGamepadTwo.circle && !prevGamepadTwo.circle) {
            arm.moveToPosition(0, 0);
        }

        telemetry.update();
    }
}
