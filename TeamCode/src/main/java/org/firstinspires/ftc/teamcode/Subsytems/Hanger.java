package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static com.qualcomm.robotcore.hardware.Servo.Direction.*;
import static org.firstinspires.ftc.teamcode.Constants.HANGER_SERVO_POSITION;

public class Hanger {

    private final Servo left, right;

    /**
     * Hanger Subsystem Constructor
     * @param opMode The opMode you are using the hanger in, likely "this"
     */
    public Hanger(@NonNull HardwareMap hardwareMap) {
        left  = hardwareMap.get(Servo.class, "lhS");
        right = hardwareMap.get(Servo.class, "rhS");
    }

    /**
     * Initializes the Hanger, this simply reverses the direction of the left servo
     */
    public void init() { left.setDirection(REVERSE); }

    /**
     * Releases the Hanging Mechanism
     */
    public void release() {
        left.setPosition(HANGER_SERVO_POSITION);
        right.setPosition(HANGER_SERVO_POSITION);
    }
}
