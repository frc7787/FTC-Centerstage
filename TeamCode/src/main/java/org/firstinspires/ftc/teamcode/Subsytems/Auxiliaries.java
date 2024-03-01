package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.HANGER_SERVO_POSITION;
import static org.firstinspires.ftc.teamcode.Properties.LAUNCHER_SERVO_LAUNCH_POS;
import static org.firstinspires.ftc.teamcode.Properties.LAUNCHER_SERVO_ZERO_POS;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Auxiliaries {
    public static double PIXEL_PLACER_SERVO_SPIKE_STRIP_POS = 0.84;
    public static double PIXEL_PLACER_SERVO_BACKDROP_POS    = 0.52;

    private static ServoImplEx hangerServo, launcherServo, pixelPlacerServoLeft, pixelPlacerServoRight;

    /**
     * Initializes all of the Auxiliary systems including the plane launcher, hanger, and pixel placer
     * servos
     * @param hardwareMap The hardware map you are using, likely "hardwareMap"
     */
    public static void init(@NonNull HardwareMap hardwareMap) {
        hangerServo           = hardwareMap.get(ServoImplEx.class, "HangerServo");
        launcherServo         = hardwareMap.get(ServoImplEx.class, "LauncherServo");
        pixelPlacerServoLeft  = hardwareMap.get(ServoImplEx.class, "PixelPlacerServoLeft");
        pixelPlacerServoRight = hardwareMap.get(ServoImplEx.class, "PixelPlacerServoRight");

        pixelPlacerServoLeft.setDirection(Servo.Direction.REVERSE);

        launcherServo.setDirection(Servo.Direction.REVERSE);
        launcherServo.setPosition(LAUNCHER_SERVO_ZERO_POS);

        hangerServo.setDirection(Servo.Direction.REVERSE);
        hangerServo.setPosition(0.0);

        pixelPlacerServoLeft.setPosition(0.1);
        pixelPlacerServoRight.setPosition(0.1);
    }

    /**
     * Releases the plane launcher
     */
    public static void releaseLauncher() {
        launcherServo.setPosition(LAUNCHER_SERVO_LAUNCH_POS);
    }

    /**
     * Resets the plane launcher
     */
    public static void resetLauncher() {
        launcherServo.setPosition(LAUNCHER_SERVO_ZERO_POS);
    }

    /**
     * Releases the hanger mechanism
     */
    public static void releaseHanger() {
        hangerServo.setPosition(HANGER_SERVO_POSITION);
    }

    /**
     * Places the left pixel placer (From the back of the robot) on the spike strip
     */
    public static void placePixelOnSpikeStripLeft() {
        pixelPlacerServoLeft.setPosition(PIXEL_PLACER_SERVO_SPIKE_STRIP_POS);
    }

    /**
     * Places the left (From the back of the robot) pixel placer on the backdrop
     */
    public static void placePixelOnBackdropLeft() {
        pixelPlacerServoLeft.setPosition(PIXEL_PLACER_SERVO_BACKDROP_POS);
    }

    /**
     * Retracts the left  (From the back of the robot) pixel placer
     */
    public static void retractPixelPlacerLeft() {
        pixelPlacerServoLeft.setPosition(0.0);
    }

    /**
     * Places the right (From the back of the robot) pixel placer on the spike strip
     */
    public static void placePixelOnSpikeStripRight() {
        pixelPlacerServoRight.setPosition(PIXEL_PLACER_SERVO_SPIKE_STRIP_POS);
    }

    /**
     * Places the right (From the back of the robot) pixel placer on the backdrop
     */
    public static void placePixelOnBackdropRight() {
        pixelPlacerServoRight.setPosition(PIXEL_PLACER_SERVO_BACKDROP_POS);
    }

    /**
     * Retracts the right (From the back of the robot) pixel placer on the backdrop
     */
    public static void retractPixelPlacerRight() {
        pixelPlacerServoRight.setPosition(0.0);
    }

    /**
     * Places the left (From the back of the robot) pixel placer in mosaic rearrangement position
     */
    public static void movePixelPlacerToMosaicFixingPositionLeft() {
        pixelPlacerServoLeft.setPosition(PIXEL_PLACER_SERVO_BACKDROP_POS + 0.1);
    }

    /**
     * Places the right (From the back of the robot) pixel placer in mosaic rearrangement position
     */
    public static void movePixelPlacerToMosaicFixingPositionRight() {
        pixelPlacerServoRight.setPosition(PIXEL_PLACER_SERVO_BACKDROP_POS + 0.1);
    }

    public static void debugHanger(@NonNull Telemetry telemetry) {
        telemetry.addLine("Hanger Debug");

        telemetry.addData("Hanger Servo Position", hangerServo.getPosition());
        telemetry.addData("Hanger Servo PWM Range", hangerServo.getPwmRange().toString());
        telemetry.addData("Hanger Servo Direction", hangerServo.getDirection());
    }

    public static void debugLauncher(@NonNull Telemetry telemetry) {
        telemetry.addLine("Launcher Debug");

        telemetry.addData("Launcher Servo Position", launcherServo.getPosition());
        telemetry.addData("Launcher Servo PWM Range", launcherServo.getPwmRange().toString());
        telemetry.addData("Launcher Servo Direction", launcherServo.getDirection());
    }

    public static void debugPixelPlacers(@NonNull Telemetry telemetry) {
        telemetry.addLine("Purple Pixel Placer Servo Debug");

        telemetry.addData("Left Pixel Placer Servo Target Position", pixelPlacerServoLeft.getPosition());
        telemetry.addData("Right Pixel Placer Servo Target Position", pixelPlacerServoRight.getPosition());
    }

    public static void debugAll(@NonNull Telemetry telemetry) {
        debugHanger(telemetry);
        debugLauncher(telemetry);
        debugPixelPlacers(telemetry);
    }
}
