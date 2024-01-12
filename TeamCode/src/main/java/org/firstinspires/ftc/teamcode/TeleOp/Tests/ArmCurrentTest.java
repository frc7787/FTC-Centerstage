package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;

@TeleOp(name = "Test - Arm Current", group = "Test")
public class ArmCurrentTest extends OpMode {
    Arm arm;

    double maxElevatorCurrent = 0.0d;
    double maxWormCurrent     = 0.0d;
    double maxArmCurrent      = 0.0d;

    public void init() {
        arm = new Arm(hardwareMap);
        arm.init();
    }

    public void loop() {
        if (arm.getElevatorCurrentAmps() > maxElevatorCurrent) {
            maxElevatorCurrent = arm.getElevatorCurrentAmps();
        }

        if (arm.getWormCurrentAmps() > maxWormCurrent) {
            maxWormCurrent = arm.getWormCurrentAmps();
        }

        if (maxWormCurrent + maxElevatorCurrent < maxArmCurrent) {
            maxArmCurrent = maxWormCurrent + maxElevatorCurrent;
        }

        telemetry.addData("Max Worm Current", maxWormCurrent);
        telemetry.addData("Max Elevator Current", maxElevatorCurrent);
        telemetry.addData("Max Arm Current", maxArmCurrent);


        if (gamepad1.left_bumper) {
            arm.moveToPosition(3045, 2326);
        } else if (gamepad1.right_bumper) {
            arm.moveToPosition(0,0);
        }

        telemetry.update();
    }
}
