package org.firstinspires.ftc.teamcode.Subsytems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Hanger {

    private final Servo left, right;
    private final Gamepad controller;

    /**
     * Hanger Subsystem Constructor
     * @param opMode The opMode you are using the hanger in, likely "this"
     */
    public Hanger(@NonNull OpMode opMode) {
        controller = opMode.gamepad1;

        left  = opMode.hardwareMap.get(Servo.class, "Left Hook Servo");
        right = opMode.hardwareMap.get(Servo.class, "Right Hook Servo");

        left.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Releases the Hanging Mechanism
     */
    public void run() {
        if (controller.left_bumper && controller.left_trigger == 1.0) {
            left.setPosition(0d);
            right.setPosition(0d);
        }
    }
}
