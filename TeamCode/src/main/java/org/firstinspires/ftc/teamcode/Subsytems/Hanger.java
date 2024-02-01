package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.Properties.HANGER_SERVO_POSITION;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to contain the hanger subsystem
 */
public class Hanger {
    private static ServoImplEx hangerServo;

    /**
     * Initializes the hanger subsystem, this sets the hanger servo to the zero position
     * and reverses the hanger servo
     */
    public static void init(@NonNull HardwareMap hardwareMap) {
        hangerServo = hardwareMap.get(ServoImplEx.class, "HangerServo");

        hangerServo.setDirection(Servo.Direction.REVERSE);
        hangerServo.setPosition(0.0);
    }

    /**
     * Moves the servo to the hanging position
     */
    public static void release() {
        hangerServo.setPosition(HANGER_SERVO_POSITION);
    }

    /**
     * Displays debug information about the hanger servo. Note to improve loop times this function
     * DOES NOT call telemetry.update()
     * @param telemetry The telemetry to display the information on
     */
    public static void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Hanger Debug");

        telemetry.addData("Hanger Servo Last Commanded Position", hangerServo.getPosition());
        telemetry.addData("Hanger Servo Direction", hangerServo.getDirection());
    }
}
