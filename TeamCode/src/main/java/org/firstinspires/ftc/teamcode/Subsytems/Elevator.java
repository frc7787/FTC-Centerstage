package org.firstinspires.ftc.teamcode.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
    private final Telemetry telemetry;
    private final Gamepad controller;
    private final TouchSensor limitSwitch;

    /**
     * Elevator Subsystem constructor
     *
     * @param opMode The OpMode that you are using the elevator in, likely "this"
     */
    public Elevator(@NonNull OpMode opMode) {
        leftExtend  = opMode.hardwareMap.get(DcMotorImplEx.class, "lExt");
        rightExtend = opMode.hardwareMap.get(DcMotorImplEx.class, "rExt");
        leftRotate  = opMode.hardwareMap.get(DcMotorImplEx.class, "lWorm");
        rightRotate = opMode.hardwareMap.get(DcMotorImplEx.class, "rWorm");

        limitSwitch = opMode.hardwareMap.get(TouchSensor.class, "lmS");

        extensionMotors = new DcMotorImplEx[]{leftExtend, rightExtend};
        rotationsMotors = new DcMotorImplEx[]{leftRotate, rightRotate};

        telemetry  = opMode.telemetry;
        controller = opMode.gamepad2;

        rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorUtility.setMode(STOP_AND_RESET_ENCODER, leftExtend, rightExtend, leftRotate, rightRotate);
    }


    public void run() {
        // This resets the encoders if the limit switch is touched
        if (limitSwitch.isPressed()) { MotorUtility.setMode(STOP_AND_RESET_ENCODER, rotationsMotors); }

        if (controller.dpad_down) { // Retracted, this is the position that the robot starts in
            rotate(0);
            extend(0);
        }
        if (controller.dpad_up) { // Fully extended on the ground
            rotate(0);
            extend(MAX_ELEVATOR_EXTENSION);
        }
        if (controller.cross) { // Rotated to the first row
            rotate(BOTTOM_ROT_POSITION);
            extend(BOTTOM_EXTEND_POSITION);
        }
        if (controller.square) { // Low Line
            rotate(LOW_ROT_POSITION);
            extend(LOW_EXTEND_POSITION);
        }
        if (controller.circle) { // Mid Line
            rotate(MED_ROT_POSITION);
            extend(MED_EXTEND_POSITION);
        }
        if (controller.triangle) { // High Line
            rotate(HIGH_ROT_POSITION);
            extend(HIGH_EXTEND_POSITION);
        }
        if (controller.options) { // Basically as High as we can go
            rotate(TOP_ROT_POSITION);
            extend(TOP_EXTEND_POSITION);
        }
    }

    public void runEndGame(@NonNull Gamepad controller) {
        if (controller.left_bumper)  { rotate(HANG_POSITION);   } // To Hang Position
        if (controller.right_bumper) { rotate(LAUNCH_POSITION); }
        if (controller.dpad_right)   { rotate(420);     }
    }

    /**
     * Moves the elevator to the desired position
     * @param position The position to extend the elevator to
     */
    private void extend(int position) {
        MotorUtility.setTargetPosition(position, extensionMotors);
        MotorUtility.setMode(RUN_TO_POSITION, extensionMotors);
        MotorUtility.setPower(0.9, extensionMotors);
    }

    /**
     * Rotates the elevator to the desired position
     * @param position The position to rotate to
     */
    private void rotate(int position) {
        MotorUtility.setTargetPosition(position, rotationsMotors);
        MotorUtility.setMode(RUN_TO_POSITION, rotationsMotors);
        MotorUtility.setPower(0.3, rotationsMotors);
    }


    public void debug() {
        telemetry.addLine("Elevator Debug");

        telemetry.addData("Target Position", leftExtend.getCurrentPosition());
        telemetry.addData("Left Motor Current Position", leftExtend.getCurrentPosition());
        telemetry.addData("Right Motor Current Position", rightExtend.getCurrentPosition());

        telemetry.addData("Left Rotation Motor Current Position", leftRotate.getCurrentPosition());
        telemetry.addData("Right Rotation Motor Current Position", rightRotate.getCurrentPosition());

        telemetry.addData("Left Rotation Motor Target Position", leftRotate.getTargetPosition());
        telemetry.addData("Right Rotation Motor Target Position", rightRotate.getTargetPosition());

        telemetry.update();
    }
}

