package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.Properties.*;

public final class Launcher {

    private final ServoImplEx launcherServo;


    public Launcher(@NonNull HardwareMap hardwareMap) {
        launcherServo = hardwareMap.get(ServoImplEx.class, "lS");
    }

    /**
     * Initializes the launcher, note that this disables the Pwm.
     */
    public void init() { launcherServo.setPwmDisable(); }


    /**
     * Releases The Launcher
     */
    public void release() {
        launcherServo.setPwmEnable();
        launcherServo.setPosition(LAUNCHER_SERVO_POSITION);
    }
}
