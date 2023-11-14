package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Hanger;

@TeleOp(name = "Test - Hang", group = "Test")
public class HangTest extends OpMode {

    public Hanger hanger;

    @Override public void init() { hanger = new Hanger(this); }

    @Override public void loop() { hanger.run(); }
}
