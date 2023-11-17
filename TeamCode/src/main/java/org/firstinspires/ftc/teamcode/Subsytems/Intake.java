package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.*;

public final class Intake {
    private final Gamepad controller;
    private final Telemetry telemetry;
    private final Servo intakeServo, wrist;

    /**
     * Intake Subsystem Constructor
     * @param opMode The opMode you are using the Intake in, likely "this"
     */
    public Intake(@NonNull OpMode opMode) {
        controller  = opMode.gamepad2;
        telemetry   = opMode.telemetry;
        intakeServo = opMode.hardwareMap.get(Servo.class, "iS");
        wrist       = opMode.hardwareMap.get(Servo.class, "wS");

        wrist.scaleRange(0.25, 0.8); // Linear servos don't do well at 1 and 0 so we cap them before then

        wrist.setPosition(0.0d);
    }

    /**
     * Runs the intake
     */
    public void run() {
        if (controller.left_bumper)           { intakeServo.setPosition(INTAKE_POSITION);  }
        if (controller.right_bumper)          { intakeServo.setPosition(HOLD_POSITION);    }
        if (controller.right_trigger == 1.0)  { intakeServo.setPosition(OUTTAKE_POSITION); }

        if (controller.dpad_down) { wrist.setPosition(0.0); }
        if (controller.dpad_up)   { wrist.setPosition(0.0); }
        if (controller.cross) { // 1st row
            wrist.setPosition(BOTTOM_WRIST_POSITION);
        }
        if (controller.square) { // Low Line
            wrist.setPosition(LOW_WRIST_POSITION);
        }
        if (controller.circle) { // Mid Line
            wrist.setPosition(MED_WRIST_POSITION);
        }
        if (controller.triangle) {
            wrist.setPosition(HIGH_WRIST_POSITION);
        }
        if (controller.options) { // Top Line
            wrist.setPosition(TOP_WRIST_POSITION);
        }

        //wrist.setPosition(controller.left_trigger);
    }

    /**
     * Puts the intake in the intake position
     */
    public void release() { intakeServo.setPosition(INTAKE_POSITION); }


    /**
     * Displays intake debug information
     */
    public void debug() {
        telemetry.addLine("Intake Debug");

        telemetry.addData("Intake Position", intakeServo.getPosition());
        telemetry.addData("Intake Direction", intakeServo.getDirection());

        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.addData("Wrist Direction", wrist.getDirection());

        telemetry.update();
     }
}
