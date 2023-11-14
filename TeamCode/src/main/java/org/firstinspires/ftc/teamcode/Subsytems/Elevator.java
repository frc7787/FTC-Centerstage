package org.firstinspires.ftc.teamcode.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.Constants.*;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Utility.MotorUtility;

/**
 * Object to encapsulate elevator subsystem
 */
public final class Elevator {

    private final DcMotorImplEx leftExtend, rightExtend, leftRotate, rightRotate;
    private final DcMotorImplEx[] extensionMotors, rotationsMotors;

    private final Gamepad controller;
    private final Telemetry telemetry;


    /**
     * Elevator Subsystem constructor
     *
     * @param opMode The OpMode that you are using the elevator in, likely "this"
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


    public void run() {
        if (controller.dpad_up)    { move_elevator(HIGH_EXTENSION_POSITION, HIGH_ROTATION_POSITION);     }
        if (controller.dpad_right) { move_elevator(MEDIUM_EXTENSION_POSITION, MEDIUM_ROTATION_POSITION); }
        if (controller.dpad_left)  { move_elevator(LOW_EXTENSION_POSITION, LOW_ROTATION_POSITION);       }
        if (controller.dpad_down)  { move_elevator(0, 0);         }
    }

    /**
     * Moves the elevator to the desired position
     * @param extensionPosition The position to extend the elevator to
     * @param rotationPosition The position to rotate the elevator to
     */
    private void move_elevator(int extensionPosition, int rotationPosition) {
        MotorUtility.setTargetPosition(rotationPosition, rotationsMotors);
        MotorUtility.setTargetPosition(extensionPosition, extensionMotors);

        MotorUtility.setMode(RUN_TO_POSITION, rotationsMotors);
        MotorUtility.setMode(RUN_TO_POSITION, extensionMotors);

        MotorUtility.setVelocity(ELEVATOR_ROTATION_VELOCITY, rotationsMotors);
        MotorUtility.setVelocity(ELEVATOR_EXTENSION_VELOCITY, extensionMotors);
    }


    public void debug() {
        telemetry.addLine("Elevator Debug");

        telemetry.addData("Target Position", leftExtend.getCurrentPosition());
        telemetry.addData("Left Motor Current Position", leftExtend.getCurrentPosition());
        telemetry.addData("Right Motor Current Position", rightExtend.getCurrentPosition());

        telemetry.addData("Left Rotation Motor Target Position", leftRotate.getTargetPosition());
        telemetry.addData("Right Rotation Motor Target Position", rightRotate.getTargetPosition());

        telemetry.update();
    }
}

