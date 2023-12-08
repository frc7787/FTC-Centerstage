package org.firstinspires.ftc.teamcode.TeleOp.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Hanger;

@TeleOp(name = "Test - Hanger", group = "Test")
@Disabled
public class HangerTest extends OpMode {

    private Hanger hanger;

    @Override public void init() {
        hanger = new Hanger(hardwareMap);
        hanger.init();
    }

    @Override public void loop() {

        if (gamepad1.triangle) { hanger.release();
        }
    }
}
