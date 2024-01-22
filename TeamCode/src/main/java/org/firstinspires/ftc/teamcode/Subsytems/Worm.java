package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
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
    final DcMotorImplEx worm;
    final TouchSensor limitSwitch;

    public Worm(@NonNull HardwareMap hardwareMap) {
        worm        = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        limitSwitch = hardwareMap.get(TouchSensor.class, "WormLimitSwitch");

        worm.setMotorEnable();
        worm.setTargetPositionTolerance(20);
    }

    /**
     * Initializes the worm subsystem. <br>
     * This resets the worm motor encoders
     */
    public void init() {
        worm.setMode(STOP_AND_RESET_ENCODER);
        worm.setMotorEnable();
    }

    /**
     * Function to update the worm every iteration of the main opMode loop. <br>
     * Checks to see if we should stop and reset encoders.
     */
    public void update() {
        if (worm.getCurrentPosition() == 0 && limitSwitch.isPressed()) {
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
    public boolean limitSwitchIsPressed() { return limitSwitch.isPressed(); }


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
     * Gets the position that the worm (rotation) motor is trying to get to
     * @return The position the worm motor is trying to get to
     */
    public int targetPos() { return worm.getTargetPosition(); }

    /**
     * @return The current pos of the robot
     */
    public int pos() { return worm.getCurrentPosition(); }

    /**
     * Displays debug information for the worm
     */

    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Worm Debug");

        telemetry.addData("Worm Direction", worm.getDirection());
        telemetry.addData("Worm Current Position", worm.getCurrentPosition());
        telemetry.addData("Worm Target Position", worm.getTargetPosition());
        telemetry.addData("Worm Current (AMPS)", worm.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Worm Limit Switch is pressed", limitSwitch.isPressed());
    }

    public double getCurrent() { return worm.getCurrent(CurrentUnit.AMPS); }

    public void disable() {
        worm.setMotorDisable();
    }
}
