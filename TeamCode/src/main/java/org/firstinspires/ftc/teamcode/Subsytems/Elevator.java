package org.firstinspires.ftc.teamcode.Subsytems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.Constants.*;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.MotorUtility;

/**
 * Object to encapsulate elevator subsystem
 */
public final class Elevator {

    private enum RotationState {
        DOWN,
        UP
    }

    private final DcMotorImplEx leftExtend, rightExtend, leftRotate, rightRotate;

    private final DcMotorImplEx[] extensionMotors, rotationsMotors;

    private final Gamepad controller;

    private final Telemetry telemetry;


    private RotationState rotationState = RotationState.DOWN;

    /**
     * Elevator constructor
     *
     * @param opMode The OpMode that you are using the elevator in, likely just "this"
     */
    public Elevator(@NonNull OpMode opMode) {
        leftExtend  = opMode.hardwareMap.get(DcMotorImplEx.class, "Left Extension Motor");
        rightExtend = opMode.hardwareMap.get(DcMotorImplEx.class, "Right Extension Motor");
        leftRotate  = opMode.hardwareMap.get(DcMotorImplEx.class, "Left Worm Motor");
        rightRotate = opMode.hardwareMap.get(DcMotorImplEx.class, "Right Worm Motor");

        extensionMotors = new DcMotorImplEx[]{leftExtend, rightExtend};
        rotationsMotors = new DcMotorImplEx[]{leftRotate, rightRotate};

        controller  = opMode.gamepad1;
        telemetry   = opMode.telemetry;

        rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorUtility.setMode(STOP_AND_RESET_ENCODER, leftExtend, rightExtend, leftRotate, rightRotate);
    }

    /**
     * Controls elevator rotation
     */
    private void rotate() {
        switch (rotationState) {
            case DOWN:
                if (controller.dpad_up) {
                    MotorUtility.setMode(RUN_TO_POSITION, rotationsMotors);
                    MotorUtility.setTargetPosition(1092, rotationsMotors);
                    MotorUtility.setVelocity(ELEVATOR_ROTATION_VELOCITY, rotationsMotors);
                    rotationState = RotationState.UP;
                }
            case UP:
                if (controller.dpad_down) {
                    MotorUtility.setMode(RUN_TO_POSITION, rotationsMotors);
                    MotorUtility.setTargetPosition(0, rotationsMotors);
                    MotorUtility.setVelocity(ELEVATOR_ROTATION_VELOCITY, rotationsMotors);
                    rotationState = RotationState.DOWN;
                }
        }
    }

    private void extend() {
        // Get target position

        if (controller.x) {
            MotorUtility.setTargetPosition(0, extensionMotors);
        } else if (controller.square) {
            MotorUtility.setTargetPosition(LOW_POSITION, extensionMotors);
        } else if (controller.circle) {
            MotorUtility.setTargetPosition(MEDIUM_POSITION, extensionMotors);
        } else if (controller.triangle) {
            MotorUtility.setTargetPosition(HIGH_POSITION, extensionMotors);
        }

        MotorUtility.setMode(RUN_TO_POSITION, extensionMotors);
        MotorUtility.setVelocity(ELEVATOR_EXTENSION_VELOCITY, extensionMotors);
    }

    /**
     * Runs the elevator
     */
    public void run() {
        rotate();
        extend();
    }


    public void debug() {
        telemetry.addLine("Elevator Debug");

        telemetry.addData("Target Position", leftExtend.getCurrentPosition());
        telemetry.addData("Left Motor Current Position", leftExtend.getCurrentPosition());
        telemetry.addData("Right Motor Current Position", rightExtend.getCurrentPosition());

        telemetry.addData("Rotation Motor Target Position", leftRotate.getTargetPosition());

        telemetry.update();
    }
}

