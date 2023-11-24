package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import static org.firstinspires.ftc.teamcode.Constants.DEFAULT_ELEVATOR_POWER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Utility.MotorUtility;

/**
 * Object to encapsulate elevator subsystem
 */
class Elevator {

    private final DcMotorImplEx leftExtend, rightExtend;
    private final DcMotorImplEx[] elevatorMotors;
    private final TouchSensor extLimitSwitch;


    public Elevator(@NonNull HardwareMap hardwareMap) {
        leftExtend     = hardwareMap.get(DcMotorImplEx.class, "lExt");
        rightExtend    = hardwareMap.get(DcMotorImplEx.class, "rExt");
        extLimitSwitch = hardwareMap.get(TouchSensor.class, "lmS");
        elevatorMotors = new DcMotorImplEx[]{leftExtend, rightExtend};
    }


    /**
     * Initializes the elevator subsystem.
     * This Resets the elevator motor encoders and reverses the polarity of the right motor.
     */
    public void init() {
        MotorUtility.setMode(STOP_AND_RESET_ENCODER, elevatorMotors);
        rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    /**
     * Checks the limit switch when the position is zero.
     * If the position is zero, and the limit switch is pressed then we set the motor powers to
     * zero and reset the encoders
     */
    public void checkLimitSwitch() {
        if (leftExtend.getCurrentPosition() == 0 && extLimitSwitch.isPressed()) {
            MotorUtility.setPower(0, elevatorMotors);

            MotorUtility.setMode(STOP_AND_RESET_ENCODER, elevatorMotors);
        }
    }

    /**
     * Extends the elevator at the defined position and speed
     * @param position The position to extend the elevator to
     * @param power The speed to extend it
     */
    public void extend(int position, double power) {
        MotorUtility.setTargetPosition(position, elevatorMotors);
        MotorUtility.setMode(RUN_TO_POSITION, elevatorMotors);
        MotorUtility.setPower(power, elevatorMotors);
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
    public void power(double power) { MotorUtility.setPower(power, elevatorMotors); }

    /**
     * Checks to see if the limit switch is pressed
     * @return Whether or not the limit switch is pressed
     */
    public boolean limitSwitchIsPressed() { return extLimitSwitch.isPressed(); }

    /**
     * Checks to see if the elevator is busy
     * @return Whether or not the elevator is busy
     */
    public boolean is_busy() { return leftExtend.isBusy() && rightExtend.isBusy(); }


    /**
     * Displays debug information about the elevator
     * @param telemetry The telemetry object you are using to display the data
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Elevator Debug");

        telemetry.addData("Target Position", leftExtend.getCurrentPosition());
        telemetry.addData("Left Motor Current Position", leftExtend.getCurrentPosition());
        telemetry.addData("Right Motor Current Position", rightExtend.getCurrentPosition());

        telemetry.addData("LS extension", extLimitSwitch.isPressed());

        telemetry.update();
    }


    /**
     * Stops the elevator by setting the power to zero
     */
    public void stop() { MotorUtility.setPower(0, elevatorMotors); }
}

