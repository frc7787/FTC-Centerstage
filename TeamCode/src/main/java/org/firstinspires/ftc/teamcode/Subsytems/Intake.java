package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Intake {
    private final Gamepad controller;

    private final Telemetry telemetry;

    private final Servo intakeServo;

    public Intake(@NonNull OpMode opMode) {
        controller  = opMode.gamepad1;
        telemetry   = opMode.telemetry;
        intakeServo = opMode.hardwareMap.get(Servo.class, "Intake Servo");
    }

    /**
     * Runs the intake
     */
    public void run() {
        if (controller.left_bumper)  { intakeServo.setPosition(INTAKE_POSITION);  }
        if (controller.right_bumper) { intakeServo.setPosition(OUTTAKE_POSITION); }
        if (controller.triangle)     { intakeServo.setPosition(HOLD_POSITION);    }
    }

    public void release() { intakeServo.setPosition(INTAKE_POSITION); }


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
