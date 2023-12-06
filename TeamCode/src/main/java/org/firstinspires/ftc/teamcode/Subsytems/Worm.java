package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_WORM_POWER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Worm {

    private final DcMotorImplEx worm;
    private final TouchSensor rotLimitSwitch;

    public Worm(@NonNull HardwareMap hardwareMap) {
        worm           = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        rotLimitSwitch = hardwareMap.get(TouchSensor.class, "WormLimitSwitch");
    }

    /**
     * Initializes the worm subsystem. <br>
     * This resets the worm motor encoders
     */
    public void init() { worm.setMode(STOP_AND_RESET_ENCODER); }

    /**
     * Function to update the worm every iteration of the main opMode loop. <br>
     * Checks to see if we should stop and reset encoders.
     */
    public void update() {
        if (worm.getCurrentPosition() == 0 && rotLimitSwitch.isPressed()) {
            worm.setMode(STOP_AND_RESET_ENCODER);
        }
    }


    /**
     * Rotates the worm to a position at the speed defined by the DEFAULT_WORM_POWER constant.
     * @param position The position to rotate to
     */
    public void rotate(int position) { rotate(position, DEFAULT_WORM_POWER); }

    /**
     * Rotates the worm to a position at the set speed
     * @param position The position to rotate to
     * @param power The power to supply to motors
     */
    public void rotate(int position, double power) {
        worm.setTargetPosition(position);
        worm.setMode(RUN_TO_POSITION);
        worm.setPower(power);
    }


    /**
     * Checks to see if the worm limit switch is pressed
     * @return Whether or not the worm limit switch is pressed
     */
    public boolean limitSwitchIsPressed() { return rotLimitSwitch.isPressed(); }


    /**
     * Checks to see if the worm drive is busy
     * @return Whether or not the worm drive is busy
     */
    public boolean is_busy() { return worm.isBusy(); }

    /**
     * Supply a power to the worm motors
     * @param power
     */
    public void power(double power) { worm.setPower(power); }

    /**
     * Gets the current position of the worm (rotation) motor
     * @return The current position of the worm (rotation) motor
     */
    public int getCurrentPosition() { return worm.getCurrentPosition(); }

    /**
     * Gets the position that the worm (rotation) motor is trying to get to
     * @return The position the worm motor is trying to get to
     */
    public int getTargetPosition() { return worm.getTargetPosition(); }
}
