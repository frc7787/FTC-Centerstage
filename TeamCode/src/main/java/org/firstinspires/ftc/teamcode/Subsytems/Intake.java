package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Properties.*;

public class Intake {
    private final CRServo left, right;

    public Intake(@NonNull HardwareMap hardwareMap) {
        left  = hardwareMap.get(CRServo.class, "LeftIntake");
        right = hardwareMap.get(CRServo.class, "RightIntake");
    }

    /**
     * Spins the intake in the intake position
     */
    public void intake() {
        left.setDirection(REVERSE);
        right.setDirection(FORWARD);
        left.setPower(INTAKE_SPEED);
        right.setPower(INTAKE_SPEED);
    }

    /**
     * Spins the intake in the outtake direction
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

        telemetry.addData("Left Intake Servo Power", left.getPower());
        telemetry.addData("Right Intake Servo Power", right.getPower());

        telemetry.addData("Left Intake Servo Direction", left.getDirection());
        telemetry.addData("Right Intake Servo Direction", right.getDirection());
     }
}
