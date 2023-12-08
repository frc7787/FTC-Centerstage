package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.Properties.*;

public class Launcher {
    private final ServoImplEx launcherServo;

    public Launcher(@NonNull HardwareMap hardwareMap) {
        launcherServo = hardwareMap.get(ServoImplEx.class, "LauncherServo");
    }

    public void init() { launcherServo.setPwmDisable(); }

    public void release() {
        launcherServo.setPwmEnable();
        launcherServo.setPosition(LAUNCHER_SERVO_POSITION);
    }

}
