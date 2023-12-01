package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.*;

class Intake {
    private final CRServo left, right;
    private final Servo intakeDoor;

    public Intake(@NonNull HardwareMap hardwareMap) {
        left  = hardwareMap.get(CRServo.class, "LeftIntakeServo");
        right = hardwareMap.get(CRServo.class, "RightIntakeServo");
        intakeDoor = hardwareMap.get(Servo.class, "IntakeDoor");
    }

    public void init() { left.setDirection(REVERSE); }


    /**
     * Sets the servo to intake
     */
    public void intake() {
        left.setDirection(REVERSE);
        right.setDirection(FORWARD);
        left.setPower(INTAKE_SPEED);
        right.setPower(INTAKE_SPEED);
    }

    /**
     * Sets servo to outtake
     */
    public void outtake() {
        left.setDirection(FORWARD);
        right.setDirection(REVERSE);
        left.setPower(OUTTAKE_SPEED);
        right.setPower(OUTTAKE_SPEED);
    }


    /**
     * Displays intake debug information
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Intake Debug");

        telemetry.addData("Left Intake Servo Direction", left.getDirection());
        telemetry.addData("Right Intake Servo Direction", right.getDirection());
        telemetry.addData("Left Intake Servo Power", left.getPower());
        telemetry.addData("Right Intake Servo Power", right.getPower());
     }
}
