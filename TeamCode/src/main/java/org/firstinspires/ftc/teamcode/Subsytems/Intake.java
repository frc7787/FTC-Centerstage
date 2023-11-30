package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.*;

class Intake {
    private final Servo intakeServo;


    public Intake(@NonNull HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(Servo.class, "iS");
    }


    /**
     * Puts the intake in the intake position
     */
    public void intake() { intakeServo.setPosition(INTAKE_POSITION); }

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
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Intake Debug");

        telemetry.addData("Intake Position", intakeServo.getPosition());
        telemetry.addData("Intake Direction", intakeServo.getDirection());

        telemetry.update();
     }
}
