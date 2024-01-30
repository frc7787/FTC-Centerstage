package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_ELEVATOR_POWER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Object to encapsulate elevator (Extension) subsystem. Note that this subsystem does not control
 * it's state. Rather, the state is controlled by the Arm subsystem which contains both the elevator,
 * and worm subsystems.
 */
public class Elevator {
    final DcMotorImplEx extendMotor;
    final TouchSensor extLimitSwitch;

    boolean extensionLimitSwitchWasPressed;

    HomingState homingState = HomingState.IDLE;

    enum HomingState {
        IDLE,
        HOMING
    }

    /**
     * Elevator Subsystem constructor
     * @param hardwareMap The hardware map you are using, likely "hardwareMap"
     */
    public Elevator(@NonNull HardwareMap hardwareMap) {
        extendMotor    = hardwareMap.get(DcMotorImplEx.class, "ExtensionMotor");
        extLimitSwitch = hardwareMap.get(TouchSensor.class, "ExtensionLimitSwitch");
    }

    /**
     * Initializes the elevator subsystem by enabling the extension motor, resetting the encoder and
     * setting the direction of the motor
     */
    public void init() {
        extendMotor.setMotorEnable();
        extendMotor.setMode(STOP_AND_RESET_ENCODER);
        extendMotor.setMode(RUN_USING_ENCODER);
        extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Function to be run every loop iteration in the main opMode. <br>
     * Checks to see if we should stop and reset encoders
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
     * Extends the elevator at the defined position and speed
     * @param position The position to extend the elevator to
     * @param power The speed to extend it
     */
    public void extend(int position, double power) {
        extendMotor.setTargetPosition(position);
        extendMotor.setMode(RUN_TO_POSITION);
        extendMotor.setPower(power);
    }

    /**
     * Extends to motors to a position at the speed defined by the DEFAULT_ELEVATOR_SPEED constant.
     * @param position The position to extend to.
     */
    public void extend(int position) { extend(position, DEFAULT_ELEVATOR_POWER); }

    /**
     * Homes the elevator. Once the elevator limit switch is pressed resets encoder and stops
     * homing
     */
    private void home() {
        if (limitSwitchIsPressed()) {
            extendMotor.setMode(STOP_AND_RESET_ENCODER);
            extendMotor.setMode(RUN_USING_ENCODER);
            homingState = HomingState.IDLE;
        } else {
            extend(0);
        }
    }

    /**
     * Tells the elevator to start homing
     */
    public void setHoming() {
        homingState = HomingState.HOMING;
    }

    /**
     * Displays debug information about the elevator
     * @param telemetry The telemetry object you are using to display the data
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Elevator Debug");

        telemetry.addData("Elevator Direction", extendMotor.getDirection());
        telemetry.addData("Elevator Current Position", extendMotor.getCurrentPosition());
        telemetry.addData("Elevator Target Position", extendMotor.getTargetPosition());
        telemetry.addData("Elevator Current (AMPS)", extendMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Elevator Limit Switch Is Pressed", extLimitSwitch.isPressed());
    }

    /**
     * @return The position that the elevator motor is trying to get to.
     */
    public int targetPos() { return extendMotor.getTargetPosition(); }

    /**
     * @return The current position of the extension motors
     */
    public int currentPos() { return extendMotor.getCurrentPosition(); }

    /**
     * Checks to see if the limit switch is pressed
     * @return Whether or not the limit switch is pressed
     */
    public boolean limitSwitchIsPressed() { return extLimitSwitch.isPressed(); }

    /**
     * Checks to see if the elevator is busy
     * @return Whether or not the elevator is busy
     */
    public boolean is_busy() { return extendMotor.isBusy(); }

    /**
     * @return The current draw of the motor in AMPS
     */
    public double currentAmps() { return extendMotor.getCurrent(CurrentUnit.AMPS); }

    /**
     * Disables the elevator motor
     */
    public void disable() {
        extendMotor.setMotorDisable();
    }

    /**
     * Enables the elevator motor
     */
    public void enable() { extendMotor.setMotorEnable(); }
}

