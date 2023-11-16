package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.*;

public final class Launcher {

    private final Gamepad controller;
    private final Servo launcherServo;

    /**
     * Launcher Subsystem Constructor
     * @param opMode The OpMode you are using the launcher in, likely "this"
     */
    public Launcher(@NonNull OpMode opMode) {
        launcherServo = opMode.hardwareMap.get(Servo.class, "Launcher Servo");
        controller    = opMode.gamepad1;
    }

    /**
     * Runs the Launcher
     */
    public void run() {
        if (controller.right_bumper && controller.right_trigger == 1.0) {
            // TO BE DONE
        }
    }
}
