package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Properties.HANGER_SERVO_POSITION;

public class Hanger {

    private final Servo hangerServo;

    /**
     * Hanger constructor
     * @param hardwareMap The hardwareMap you are using, likely "hardwareMap"
     */
    public Hanger(@NonNull HardwareMap hardwareMap) {
       hangerServo = hardwareMap.get(Servo.class, "HangerServo");
    }

    /**
     * Moves the servo to the hanging position
     */
    public void release() {
        hangerServo.setPosition(HANGER_SERVO_POSITION);
    }
}
