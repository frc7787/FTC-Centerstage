package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.HANGER_SERVO_POSITION;
import static org.firstinspires.ftc.teamcode.Properties.LAUNCHER_SERVO_LAUNCH_POS;
import static org.firstinspires.ftc.teamcode.Properties.LAUNCHER_ZERO_POS;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Auxiliaries {
    private static ServoImplEx hangerServo, launcherServo, purplePixelPlacerServo;

    public static void init(@NonNull HardwareMap hardwareMap) {
        hangerServo = hardwareMap.get(ServoImplEx.class, "HangerServo");
        launcherServo = hardwareMap.get(ServoImplEx.class, "LauncherServo");
        purplePixelPlacerServo = hardwareMap.get(ServoImplEx.class, "PurplePixelPlacerServo");

        launcherServo.setDirection(Servo.Direction.REVERSE);
        launcherServo.setPosition(LAUNCHER_ZERO_POS);

        hangerServo.setDirection(Servo.Direction.REVERSE);
        hangerServo.setPosition(0.0);
    }

    public static void releaseLauncher() {
        launcherServo.setPosition(LAUNCHER_SERVO_LAUNCH_POS);
    }

    public static void releaseHanger() {
        hangerServo.setPosition(HANGER_SERVO_POSITION);
    }

    public static void releasePurplePixelPlacer() {
        purplePixelPlacerServo.setPosition(1.0);
    }

    public static void retractPurplePixelPlacer() {
        purplePixelPlacerServo.setPosition(0.0);
    }

    public static void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Hanger Debug");

        telemetry.addData("Hanger Servo Position", hangerServo.getPosition());
        telemetry.addData("Hanger Servo PWM Range", hangerServo.getPwmRange());
        telemetry.addData("Hanger Servo Direction", hangerServo.getDirection());

        telemetry.addLine("Launcher Debug");

        telemetry.addData("Launcher Servo Position", launcherServo.getPosition());
        telemetry.addData("Launcher Servo PWM Range", launcherServo.getPwmRange());
        telemetry.addData("Launcher Servo Direction", launcherServo.getDirection());

        telemetry.addLine("Purple Pixel Placer Servo Debug");

        telemetry.addData("Purple Pixel Placer Servo Position", purplePixelPlacerServo.getPosition());
        telemetry.addData("Purple Pixel Placer Servo PWN Range", purplePixelPlacerServo.getPwmRange());
        telemetry.addData("Purple Pixel Placer Servo Direction", purplePixelPlacerServo.getDirection());
    }
}
