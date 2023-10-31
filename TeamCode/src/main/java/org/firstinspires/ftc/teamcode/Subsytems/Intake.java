package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.Constants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.ServoUtility;

public class Intake {

    private enum IntakeState {
        IDLE,
        GRAB,
        INTAKE
    }

    private IntakeState intakeState = IntakeState.IDLE;

    private final Gamepad controller;

    private final Telemetry telemetry;

    private final CRServo left, right;

    private final CRServo[] servos;

    public Intake(@NonNull OpMode opMode) {
        controller = opMode.gamepad1;
        telemetry  = opMode.telemetry;

        left  = opMode.hardwareMap.get(CRServo.class, "Left Intake Servo");
        right = opMode.hardwareMap.get(CRServo.class, "Right Intake Servo");

        servos = new CRServo[]{left, right};

        right.setDirection(CRServo.Direction.REVERSE);
    }

    public void run() {
        if (controller.left_bumper && controller.right_bumper) { return; }

        switch(intakeState) {
            case IDLE:
                if (controller.left_bumper) {
                    intakeState = IntakeState.GRAB;
                    break;
                } else if (controller.right_bumper) {
                    intakeState = IntakeState.INTAKE;
                    break;
                }
            case INTAKE:
                if (controller.right_bumper) {
                    intake();
                    break;
                } else if (controller.left_bumper) {
                    intakeState = IntakeState.GRAB;
                    break;
                }
                intakeState = IntakeState.IDLE;
            case GRAB:
                if (controller.left_bumper) {
                    grab();
                } else if (controller.right_bumper) {
                    intakeState = IntakeState.INTAKE;
                    break;
                }
                intakeState = IntakeState.IDLE;
        }
    }

     private void intake() { ServoUtility.setPower(INTAKE_SPEED, servos); }

     private void grab() { ServoUtility.setPower(-INTAKE_SPEED, servos); }

    /**
     * Displays intake debug information
     */
    public void debug() {
        telemetry.addLine("Intake Debug");

        telemetry.addData("Left Intake Power", left.getPower());
        telemetry.addData("Right Intake Power", right.getPower());

        telemetry.addData("Left Intake Direction", left.getDirection());
        telemetry.addData("Right Intake Direction", right.getDirection());

        telemetry.update();
     }
}
