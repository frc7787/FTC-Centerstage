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
        controller  = opMode.gamepad1;
        telemetry   = opMode.telemetry;
        intakeServo = opMode.hardwareMap.get(Servo.class, "iS");
        wrist       = opMode.hardwareMap.get(Servo.class, "wS");
    }

    /**
     * Runs the intake
     */
    public void run() {
        if (controller.left_bumper)           { intakeServo.setPosition(INTAKE_POSITION);  }
        if (controller.right_bumper)          { intakeServo.setPosition(HOLD_POSITION);    }
        if (controller.right_trigger == 1.0)  { intakeServo.setPosition(OUTTAKE_POSITION); }


        if (controller.x) { // 1st row
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
    }

    /**
     * Puts the intake in the release position
     */
    public void release() { intakeServo.setPosition(INTAKE_POSITION); }

    /**
     * Puts the intake in the hold position
     */
    public void hold() { intakeServo.setPosition(HOLD_POSITION); }

    /**
     * Puts the intake in the outtake position
     */
    public void outtake() { intakeServo.setPosition(OUTTAKE_POSITION); }


    /**
     * Displays intake debug information
     */
    public void debug() {
        telemetry.addLine("Intake Debug");

        telemetry.addData("Intake Position", intakeServo.getPosition());
        telemetry.addData("Intake Direction", intakeServo.getDirection());

        telemetry.update();
     }
}
