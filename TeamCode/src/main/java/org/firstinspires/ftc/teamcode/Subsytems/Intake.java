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

    public void intake() { intakeTest(INTAKE_SPEED); }

    public void intakeTest(double intakeSpeed) {
        left.setDirection(REVERSE);
        right.setDirection(FORWARD);
        left.setPower(intakeSpeed);
        right.setPower(intakeSpeed);
    }

    public void outtake() { outtakeTest(OUTTAKE_SPEED); }

    public void outtakeTest(double outtakeSpeed) {
        left.setDirection(FORWARD);
        right.setDirection(REVERSE);
        left.setPower(outtakeSpeed);
        right.setPower(outtakeSpeed);
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
