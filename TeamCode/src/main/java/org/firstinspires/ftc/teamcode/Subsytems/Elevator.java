package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
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
    final DcMotorImplEx extend;
    final TouchSensor extLimitSwitch;

    boolean extensionLimitSwitchWasPressed;

    /**
     * Elevator Subsystem constructor
     * @param hardwareMap The hardware map you are using, likely "hardwareMap"
     */
    public Elevator(@NonNull HardwareMap hardwareMap) {
        extend         = hardwareMap.get(DcMotorImplEx.class, "ExtensionMotor");
        extLimitSwitch = hardwareMap.get(TouchSensor.class, "ExtensionLimitSwitch");

        extend.setMotorEnable();
        extend.setTargetPositionTolerance(20);

        extend.setTargetPosition(0);
        extend.setMode(RUN_TO_POSITION);
        extend.setPower(0.0);
    }


    /**
     * Initializes the elevator subsystem. <br>
     * This resets the elevator motor encoder
     */
    public void init() {
        extend.setMode(STOP_AND_RESET_ENCODER);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    /**
     * Function to be run every loop iteration in the main opMode. <br>
     * Checks to see if we should stop and reset encoders
     */
    public void update() {
        if (extLimitSwitch.isPressed() && !extensionLimitSwitchWasPressed && extend.getTargetPosition() == 0) {
            extend.setMode(STOP_AND_RESET_ENCODER);
        }

        extensionLimitSwitchWasPressed = extLimitSwitch.isPressed();
    }

    /**
     * Extends the elevator at the defined position and speed
     * @param position The position to extend the elevator to
     * @param power The speed to extend it
     */
    public void extend(int position, double power) {
        extend.setTargetPosition(position);
        extend.setMode(RUN_TO_POSITION);
        extend.setPower(power);
    }

    /**
     * Extends to motors to a position at the speed defined by the DEFAULT_ELEVATOR_SPEED constant.
     * @param position The position to extend to.
     */
    public void extend(int position) { extend(position, DEFAULT_ELEVATOR_POWER); }

    /**
     * Powers the elevator motors
     * @param power The power to supply the elevator motors
     */
    public void power(double power) { extend.setPower(power); }

    /**
     * Checks to see if the limit switch is pressed
     * @return Whether or not the limit switch is pressed
     */
    public boolean limitSwitchIsPressed() { return extLimitSwitch.isPressed(); }

    /**
     * Checks to see if the elevator is busy
     * @return Whether or not the elevator is busy
     */
    public boolean is_busy() { return extend.isBusy(); }

    /**
     * Displays debug information about the elevator
     * @param telemetry The telemetry object you are using to display the data
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Elevator Debug");

        telemetry.addData("Elevator Direction", extend.getDirection());
        telemetry.addData("Elevator Current Position", extend.getCurrentPosition());
        telemetry.addData("Elevator Target Position", extend.getTargetPosition());
        telemetry.addData("Elevator Current (AMPS)", extend.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Elevator Limit Switch Is Pressed", extLimitSwitch.isPressed());
    }

    /**
     * @return The position that the elevator motor is trying to get to.
     */
    public int targetPos() { return extend.getTargetPosition(); }

    public int pos() { return extend.getCurrentPosition(); }

    /**
     * Disables the motor
     */
    public void disable() {
        extend.setMotorDisable();
    }

    /**
     * @return The current draw of the motor in AMPS
     */
    public double currentAmps() { return extend.getCurrent(CurrentUnit.AMPS); }
}

