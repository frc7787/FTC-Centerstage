package org.firstinspires.ftc.teamcode.TeleOp.Tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test - Servo", group = "Test")
public class ServoTest extends OpMode {

    public Servo left, right;

    public void init() {
        left  = hardwareMap.get(Servo.class, "Left Hook Servo");
        right = hardwareMap.get(Servo.class, "Right Hook Servo");
        left.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() {
        left.setPosition(1d);
        right.setPosition(1d);
    }
}
