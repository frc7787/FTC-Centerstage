package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import static org.firstinspires.ftc.teamcode.Constants.DEFAULT_WORM_POWER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.TeleOp.Utility.MotorUtility;

class Worm {
    public final DcMotorImplEx leftWorm, rightWorm;
    public final TouchSensor rotLimitSwitch;
    public final DcMotorImplEx[] wormMotors;

    public Worm(@NonNull HardwareMap hardwareMap) {
        leftWorm       = hardwareMap.get(DcMotorImplEx.class, "lWorm");
        rightWorm      = hardwareMap.get(DcMotorImplEx.class, "rWorm");
        rotLimitSwitch = hardwareMap.get(TouchSensor.class, "lmS2");

        wormMotors = new DcMotorImplEx[]{leftWorm, rightWorm};
    }

    /**
     * Constantly checks to see if the worm limit switch is being pressed.
     * If it is being pressed, stops the motors, then resets the encoders
     */
    public void checkLimitSwitch() {
        if (leftWorm.getCurrentPosition() == 0 && rotLimitSwitch.isPressed()) {
            MotorUtility.setPower(0, wormMotors);
            MotorUtility.setMode(STOP_AND_RESET_ENCODER, wormMotors);
        }
    }

    /**
     * Initializes the motors.
     * This resets the left and right worm encoders
     */
    public void init() { MotorUtility.setMode(STOP_AND_RESET_ENCODER, wormMotors); }


    /**
     * Rotates the worm to a position at the speed defined by the DEFAULT_WORM_POWER constant.
     * @param position The position to rotate to
     */
    public void rotate(int position) { rotate(position, DEFAULT_WORM_POWER); }


    /**
     * Checks to see if the worm limit switch is pressed
     * @return Whether or not the worm limit switch is pressed
     */
    public boolean limitSwitchIsPressed() { return rotLimitSwitch.isPressed(); }


    /**
     * Rotates the worm to a position at the set speed
     * @param position The position to rotate to
     * @param power The power to supply to motors
     */
    public void rotate(int position, double power) {
        MotorUtility.setTargetPosition(position, wormMotors);
        MotorUtility.setMode(RUN_TO_POSITION, wormMotors);
        MotorUtility.setPower(power, wormMotors);
    }

    /**
     * Checks to see if the worm drive is busy
     * @return Whether or not the worm drive is busy
     */
    public boolean is_busy() { return leftWorm.isBusy() && rightWorm.isBusy(); }

    /**
     * Supply a power to the worm motors
     * @param power
     */
    public void power(double power) { MotorUtility.setPower(power, wormMotors); }

    /**
     * Stop to worm drive
     */
    public void stop() { MotorUtility.setPower(0, wormMotors); }
}
