package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.Properties.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    private final ServoImplEx launcherServo;

    public Launcher(@NonNull HardwareMap hardwareMap) {
        launcherServo = hardwareMap.get(ServoImplEx.class, "LauncherServo");
    }

    /**
     * Disables the launcher Pwm so that it doesn't fire
     */
    public void init() { launcherServo.setPwmDisable(); }

    /**
     * Releases the launcer servo
     */
    public void release() {
        launcherServo.setPwmEnable();
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
