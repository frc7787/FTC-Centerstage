package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Properties.HANGER_SERVO_POSITION;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hanger {
    Servo hangerServo;

    /**
     * Hanger constructor
     * @param hardwareMap The hardwareMap you are using, likely "hardwareMap"
     */
    public Hanger(@NonNull HardwareMap hardwareMap) {
       hangerServo = hardwareMap.get(Servo.class, "HangerServo");
    }

    /**
     * Initializes the hanger subsystem, this sets the hanger servo to the zero position
     */
    public void zero() {
        hangerServo.setPosition(0.0);
    }

    /**
     * Moves the servo to the hanging position
     */
    public void release() {
        hangerServo.setPosition(HANGER_SERVO_POSITION);
    }

    /**
     * Displays debug information about the hanger servo
     * @param telemetry The telemetry to display the information on
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addData("Hanger Servo Last Commanded Position", hangerServo.getPosition());
        telemetry.addData("Hanger Servo Direction", hangerServo.getDirection());
    }
}
