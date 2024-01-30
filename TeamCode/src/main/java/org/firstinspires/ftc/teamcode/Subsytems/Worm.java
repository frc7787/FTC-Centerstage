package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_WORM_POWER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Class to represent worm (rotation) subsystem.
 * It should be noted that this class does not contain state. That state is contained in the arm
 * subsystem which uses both this subsystem, and the elevator subsystem.
 */
public class Worm {
    final DcMotorImplEx wormMotor;
    final TouchSensor limitSwitch;

    boolean limitSwitchWasPressed = false;

    HomingState homingState = HomingState.IDLE;

    private enum HomingState {
        IDLE,
        HOMING
    }

    public Worm(@NonNull HardwareMap hardwareMap) {
        wormMotor   = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        limitSwitch = hardwareMap.get(TouchSensor.class, "WormLimitSwitch");
    }

    /**
     * Initializes the worm subsystem. <br>
     * This resets the worm motor encoders
     */
    public void init() {
        wormMotor.setMode(STOP_AND_RESET_ENCODER);
        wormMotor.setMotorEnable();
    }

    /**
     * Function to update the worm every iteration of the main opMode loop. <br>
     * Checks to see if we should stop and reset encoders.
     */
    public void update() {
        switch (homingState) {
            case IDLE:
                break;
            case HOMING:
                home();
        }
    }

    /**
     * Rotates the worm to a position at the set speed
     * @param position The position to rotate to
     * @param power The power to supply to motors
     */
    public void rotate(int position, double power) {
        wormMotor.setTargetPosition(position);
        wormMotor.setMode(RUN_TO_POSITION);
        wormMotor.setPower(power);
    }

    /**
     * Rotates the worm to a position at the speed defined by the DEFAULT_WORM_POWER constant.
     * @param position The position to rotate to
     */
    public void rotate(int position) { rotate(position, DEFAULT_WORM_POWER); }

    private void home() {
        if (limitSwitchIsPressed()) {
            wormMotor.setMode(STOP_AND_RESET_ENCODER);
            wormMotor.setMode(RUN_USING_ENCODER);
            homingState = HomingState.IDLE;
        } else {
            rotate(0);
        }
    }

    public void setHoming() {
        homingState = HomingState.HOMING;
    }


    /**
     * Checks to see if the worm limit switch is pressed
     * @return Whether or not the worm limit switch is pressed
     */
    public boolean limitSwitchIsPressed() { return limitSwitch.isPressed(); }

    /**
     * Checks to see if the worm drive is busy
     * @return Whether or not the worm drive is busy
     */
    public boolean is_busy() { return wormMotor.isBusy(); }

    /**
     * Supply a power to the worm motors
     * @param power
     */
    public void power(double power) { wormMotor.setPower(power); }

    /**
     * Gets the position that the worm (rotation) motor is trying to get to
     * @return The position the worm motor is trying to get to
     */
    public int targetPos() { return wormMotor.getTargetPosition(); }

    /**
     * @return The current position of the worm motor
     */
    public int currentPos() { return wormMotor.getCurrentPosition(); }

    /**
     * @return The current draw of the worm motor in AMPS
     */
    public double currentAmps() { return wormMotor.getCurrent(CurrentUnit.AMPS); }

    /**
     * Displays debug information for the worm. Note to help increase loop times this function
     * DOES NOT call telemetry.update()
     * @param telemetry The telemetry to display the debug information on
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Worm Debug");

        telemetry.addData("Worm Direction", wormMotor.getDirection());
        telemetry.addData("Worm Current Position", wormMotor.getCurrentPosition());
        telemetry.addData("Worm Target Position", wormMotor.getTargetPosition());
        telemetry.addData("Worm Current (AMPS)", wormMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Worm Limit Switch is pressed", limitSwitch.isPressed());
    }

    /**
     * Disables the worm motor
     */
    public void disable() {
        wormMotor.setMotorDisable();
    }

    /**
     * Enables the worm motor
     */
    public void enable() { wormMotor.setMotorEnable(); }
}
