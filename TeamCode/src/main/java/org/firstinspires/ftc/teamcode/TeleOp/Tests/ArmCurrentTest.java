package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import static org.firstinspires.ftc.teamcode.Properties.TOP_ROT_POS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;

@TeleOp(name = "Test - Arm Current", group = "Test")
public class ArmCurrentTest extends OpMode {

    private Arm arm;

    private double maxElevatorCurrent = 0.0;
    private double maxWormCurrent     = 0.0;

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

        telemetry.addData("Max Worm Current", maxWormCurrent);
        telemetry.addData("Max Elevator Current", maxElevatorCurrent);


        if (gamepad1.left_bumper) {
            arm.moveToPosition(3045, 2326);
        } else if (gamepad1.right_bumper) {
            arm.moveToPosition(0,0);
        }

        telemetry.update();
    }
}
