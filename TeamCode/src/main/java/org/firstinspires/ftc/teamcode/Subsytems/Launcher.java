package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.Properties.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    final ServoImplEx launcherServo;

    public Launcher(@NonNull HardwareMap hardwareMap) {
        launcherServo = hardwareMap.get(ServoImplEx.class, "LauncherServo");
    }

    public void init() {
        launcherServo.setDirection(REVERSE);
        launcherServo.setPosition(LAUNCHER_ZERO_POSITION);
    }

    /**
     * Sets the launcher servo to the zero position
     */
    public void zero() {
        launcherServo.setPosition(LAUNCHER_ZERO_POSITION);
    }

    /**
     * Releases the launcher servo
     */
    public void release() {
        launcherServo.setPosition(LAUNCHER_SERVO_POSITION);
    }

    /**
     * Displays debug information about the launcher servo
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Launcher Debug");

        telemetry.addData("Launcher Servo Last Commanded Position", launcherServo.getPosition());
        telemetry.addData("Launcher Servo Direction", launcherServo.getDirection());
    }
}
