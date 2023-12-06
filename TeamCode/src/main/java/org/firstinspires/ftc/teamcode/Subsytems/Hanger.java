package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static com.qualcomm.robotcore.hardware.Servo.Direction.*;
import static org.firstinspires.ftc.teamcode.Properties.HANGER_SERVO_POSITION;

public class Hanger {

    private final Servo left, right;

    /**
     * Hanger constructor
     * @param hardwareMap The hardwareMap you are using, likely "hardwareMap"
     */
    public Hanger(@NonNull HardwareMap hardwareMap) {
        left  = hardwareMap.get(Servo.class, "LeftHangerServo");
        right = hardwareMap.get(Servo.class, "RightHangerServo");
    }

    /**
     * Initializes the Hanger, this simply reverses the direction of the left servo
     */
    public void init() { left.setDirection(REVERSE); }


    /**
     * Releases the hangers
     */
    public void release() {
        left.setPosition(HANGER_SERVO_POSITION);
        right.setPosition(HANGER_SERVO_POSITION);
    }
}
