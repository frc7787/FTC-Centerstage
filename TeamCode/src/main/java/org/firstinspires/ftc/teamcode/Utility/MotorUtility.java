package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import static com.qualcomm.robotcore.hardware.DcMotorImplEx.*;

import androidx.annotation.NonNull;

public class MotorUtility {

    /**
     * Sets the run mode of the supplied motors
     * @param runMode The mode to run the motors in
     * @param motors  The motors to set the mode of
     */
    public static void setMode(@NonNull RunMode runMode, @NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) { motor.setMode(runMode); }
    }

    /**
     * Sets the zero power behavior of the supplied motors
     * @param zeroPowerBehavior The zero power behaviour to set the motors to
     * @param motors            The motors you are setting the zero power behaviour of
     */
    public static void setZeroPowerBehaviour(@NonNull ZeroPowerBehavior zeroPowerBehavior, @NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) { motor.setZeroPowerBehavior(zeroPowerBehavior); }
    }

    /**
     * Sets the direction of the supplied motors
     * @param direction The direction to set the motors
     * @param motors    The motors you are setting the direction of
     */
    public static void setDirection(@NonNull Direction direction, @NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) { motor.setDirection(direction); }
    }

    /**
     * Sets the target power of the supplied motors
     *
     * @param Power The power of the motors
     * @param motors The motors to set the power of
     */
    public static void setPower(double power, @NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) { motor.setPower(power); }
    }
}
