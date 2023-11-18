package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Zero Worm", group = "Utility")
public class ZeroWorm extends OpMode {

    public DcMotorEx left, right;
    public TouchSensor limitSwitch;

    @Override public void init() {
        left = hardwareMap.get(DcMotorEx.class, "lWorm");
        right = hardwareMap.get(DcMotorEx.class, "rWorm");

        limitSwitch = hardwareMap.get(TouchSensor.class, "lmS");
    }

    @Override public void loop() {
        left.setPower(-0.3);
        right.setPower(-0.3);

        if (limitSwitch.isPressed()) {
            left.setPower(0);
            right.setPower(0);
        }
    }

}
