package org.firstinspires.ftc.teamcode.Subsytems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Hanger {

    private final Servo left, right;

    private final Gamepad controller;

    public Hanger(@NonNull OpMode opMode) {
        controller = opMode.gamepad1;

        left  = opMode.hardwareMap.get(Servo.class, "Left Hanger Servo");
        right = opMode.hardwareMap.get(Servo.class, "Right Hanger Servo");

        left.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Releases the Hanging Mechanism
     */
    public void run() {
        if (controller.options) {
            left.setPosition(0.5d);
            right.setPosition(0.5d);
        }
    }
}
