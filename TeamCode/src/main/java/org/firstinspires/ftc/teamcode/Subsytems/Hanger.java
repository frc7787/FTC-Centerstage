package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import static com.qualcomm.robotcore.hardware.Servo.Direction.*;

public class Hanger {

    private final Servo left, right;
    private final Gamepad controller;


    /**
     * Hanger Subsystem Constructor
     * @param opMode The opMode you are using the hanger in, likely "this"
     */
    public Hanger(@NonNull OpMode opMode) {
        controller = opMode.gamepad2;

        left  = opMode.hardwareMap.get(Servo.class, "lhS");
        right = opMode.hardwareMap.get(Servo.class, "rhS");

        left.setDirection(REVERSE);
    }

    /**
     * Releases the Hanging Mechanism
     */
    public void run() {
        if (controller.cross) {
            left.setPosition(1.0d);
            right.setPosition(1.0d);
        }
    }
}
