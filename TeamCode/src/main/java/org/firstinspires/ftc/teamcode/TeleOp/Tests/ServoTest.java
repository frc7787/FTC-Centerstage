package org.firstinspires.ftc.teamcode.TeleOp.Tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test - Servo", group = "Test")
public class ServoTest extends OpMode {

    public Servo left, right;


    @Override public void init() {
        left  = hardwareMap.get(Servo.class, "lhS");
        right = hardwareMap.get(Servo.class, "rhS");
        left.setDirection(Servo.Direction.REVERSE);
    }

    @Override public void loop() {
        left.setPosition(1.0d);
        right.setPosition(1.0d);
    }
}
