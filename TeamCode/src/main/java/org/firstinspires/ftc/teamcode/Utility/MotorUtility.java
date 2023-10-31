package org.firstinspires.ftc.teamcode.Utility;


import static com.qualcomm.robotcore.hardware.DcMotorImplEx.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class MotorUtility {

    /**
     * Sets the run mode of the supplied motors
     * @param runMode The mode that you would like to run the motors in. All motors will be set the same.
     * @param motors  The motors that you would like to pass in, you can either pass in an array or individual arguments
     */
    public static void setMode(@NonNull RunMode runMode, @NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) { motor.setMode(runMode); }
    }

    /**
     * Sets the zero power behavior of the supplied motors
     * @param zeroPowerBehavior The zero power behavior that the motors should run in. All motors will be set the same.
     * @param motors The motors you are modifying, you can either pass in an array or individual arguments
     */
    public static void setZeroPowerBehaviour(@NonNull ZeroPowerBehavior zeroPowerBehavior, @NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) { motor.setZeroPowerBehavior(zeroPowerBehavior); }
    }

    /**
     * Sets the direction of the supplied motors
     * @param direction The direction to set the motors. All motors will be set the same.
     * @param motors The motors you are modifying, you can either pass in an array or individual arguments
     */
    public static void setDirection(@NonNull Direction direction, @NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) { motor.setDirection(direction); }
    }

    /**
     * Sets the target position of the supplied motors
     * @param targetPosition The target position to set the motors to. All motors will be set the same
     * @param motors The motors you are modifying, you can either pass in an array or individual arguments
     */
    public static void setTargetPosition(int targetPosition, @NonNull DcMotorEx ... motors) {
        for (DcMotorEx motor : motors) { motor.setTargetPosition(targetPosition); }
    }

    /**
     * Sets the velocity of the supplied motors
     * @param velocity The velocity you would like to set the motors to; Ticks/Second
     * @param motors The motors you are modifying, you can either pass in an array or individual arguments
     */
    public static void setVelocity(int velocity, @NonNull DcMotorEx ... motors) {
        for (DcMotorEx motor : motors) { motor.setVelocity(velocity); }
    }
}
