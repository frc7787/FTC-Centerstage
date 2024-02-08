package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.Properties.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem to contain the drone launcher.
 */
public class Launcher {
    private static ServoImplEx launcherServo;

    /**
     * Initializes the servo by setting the servo direction to reverse, and setting the launcher servo
     * to the position defined by the LAUNCHER_ZERO_POS
     */
    public static void init(@NonNull HardwareMap hardwareMap) {
        // Control Hub Servo Port 2
        launcherServo = hardwareMap.get(ServoImplEx.class, "LauncherServo");

        launcherServo.setDirection(REVERSE);
        launcherServo.setPosition(LAUNCHER_ZERO_POS);
    }

    /**
     * Sets the launcher servo to the zero position
     */
    public static void zero() {
        launcherServo.setPosition(LAUNCHER_ZERO_POS);
    }

    /**
     * Releases the launcher servo
     */
    public static void release() {
        launcherServo.setPosition(LAUNCHER_SERVO_LAUNCH_POS);
    }

    /**
     * Displays debug information about the launcher servo. Note to help increase loop time this
     * function DOES NOT call telemetry.update()
     *
     * @param telemetry The telemetry to display the debug information on
     */
    public static void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Launcher Debug");

        telemetry.addData("Launcher Servo Last Commanded Position", launcherServo.getPosition());
        telemetry.addData("Launcher Servo Direction", launcherServo.getDirection());
    }
}
