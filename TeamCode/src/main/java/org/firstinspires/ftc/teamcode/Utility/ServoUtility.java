package org.firstinspires.ftc.teamcode.Utility;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.robotcore.hardware.CRServo;


public abstract class ServoUtility {

    public static void setDirection(@NonNull Direction direction, @NonNull CRServo ... servos) {
        for (CRServo servo : servos) { servo.setDirection(direction); }
    }

    public static void setPower(double power, @NonNull CRServo ... servos) {
        for (CRServo servo : servos) { servo.setPower(power); }
    }
}
