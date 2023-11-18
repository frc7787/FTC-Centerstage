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
    private final TouchSensor extLimitSwitch, rotLimitSwitch;
    private boolean checkRotLimitSwitch, checkExtLimitSwitch = true;
    private double defaultPower=0.3;


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

        extLimitSwitch = opMode.hardwareMap.get(TouchSensor.class, "lmS");
        rotLimitSwitch = opMode.hardwareMap.get(TouchSensor.class, "rmS");

        extensionMotors = new DcMotorImplEx[]{leftExtend, rightExtend};
        rotationsMotors = new DcMotorImplEx[]{leftRotate, rightRotate};

        telemetry  = opMode.telemetry;
        controller = opMode.gamepad2;

        rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorUtility.setMode(STOP_AND_RESET_ENCODER, leftExtend, rightExtend, leftRotate, rightRotate);
        init();
    }


    public void init(){
        MotorUtility.setPower(0, rotationsMotors);
        MotorUtility.setPower(0, extensionMotors);

    }
    public void run(boolean l_EndGame) {
        // This resets the encoders if the limit switch is touched
        if (checkExtLimitSwitch && extLimitSwitch.isPressed()) {
            leftExtend.setPower(0);
            rightExtend.setPower(0);

            leftExtend.setMode(STOP_AND_RESET_ENCODER);
            rightExtend.setMode(STOP_AND_RESET_ENCODER);

            checkExtLimitSwitch = false;
        }

        if (checkRotLimitSwitch && rotLimitSwitch.isPressed()) {
            leftRotate.setPower(0);
            rightRotate.setPower(0);

            leftRotate.setMode(STOP_AND_RESET_ENCODER);
            rightRotate.setMode(STOP_AND_RESET_ENCODER);

            checkRotLimitSwitch = false;
        }

        if(!l_EndGame) {//run this codeonly before endgame
            if (controller.dpad_down) { // Retracted, this is the position that the robot starts in
                extend(0);
                rotate(0);

            }
            if (controller.dpad_up) { // Fully extended on the ground
                extend(MED_EXTEND_POSITION);
                rotate(0);
            }
            if (controller.cross) { // Rotated to the first row
                extend(BOTTOM_EXTEND_POSITION);
                rotate(BOTTOM_ROT_POSITION);
            }
            if (controller.square) { // Low Line
                extend(LOW_EXTEND_POSITION);
                rotate(LOW_ROT_POSITION);
            }
            if (controller.circle) { // Mid Line
                extend(LOW_ROT_POSITION);
                rotate(MED_ROT_POSITION);
            }
            if (controller.triangle) { // High Line
                extend(HIGH_EXTEND_POSITION);
                rotate(HIGH_ROT_POSITION);
            }
            if (controller.options) { // Basically as High as we can go
                extend(HIGH_ROT_POSITION);
                rotate(TOP_ROT_POSITION);
            }
        }
        else {//run this code only in endgme
            if (controller.left_bumper) {
                rotate(HANG_POSITION, 0.7);
            } // To Hang Position
            else if (controller.right_bumper) {
                rotate(LAUNCH_POSITION);
            }
            else if (controller.dpad_down) {
                rotate(420, 0.7);
            }
        }
    }





//    public void runEndGame(@NonNull Gamepad controller) {
//        if (controller.left_bumper)  { rotate(HANG_POSITION, 0.7);   } // To Hang Position
//        if (controller.right_bumper) { rotate(LAUNCH_POSITION); }
//        if (controller.dpad_down)    { rotate(420, 0.7);     }
//    }

    /**
     * Moves the elevator to the desired position
     * @param position The position to extend the elevator to
     */
    public void extend(int position) {
        if (position==0){
            checkExtLimitSwitch=true;
        }
        MotorUtility.setTargetPosition(position, extensionMotors);
        MotorUtility.setMode(RUN_TO_POSITION, extensionMotors);
        MotorUtility.setPower(0.9, extensionMotors);
    }

    /**
     * Rotates the elevator to the desired position
     * @param position The position to rotate to
     */
    private void rotate(int position) {
        rotate(position,defaultPower);
    }

    private void rotate(int position, double power) {
        if (position == 0 ) {
            checkExtLimitSwitch=true;
        }
        MotorUtility.setTargetPosition(position, rotationsMotors);
        MotorUtility.setMode(RUN_TO_POSITION, rotationsMotors);
        MotorUtility.setPower(power, rotationsMotors);
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
        telemetry.addData("zeroing elevator", checkExtLimitSwitch);
        telemetry.addData("zeroing rotation", checkRotLimitSwitch);

        telemetry.update();
    }
}

