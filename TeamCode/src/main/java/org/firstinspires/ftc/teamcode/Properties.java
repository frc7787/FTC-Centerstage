package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Properties {
    // ---------- ARM PROPERTIES -- //

    public static volatile int BOTTOM_EXT_POS = 1580;
    public static volatile int BOTTOM_ROT_POS = 1250;

    public static volatile int LOW_EXT_POS = 1820;
    public static volatile int LOW_ROT_POS = 1470;

    public static volatile int MED_EXT_POS = 2401;
    public static volatile int MED_ROT_POS = 1814;

    public static volatile int HIGH_EXT_POS = 2894;
    public static volatile int HIGH_ROT_POS = 1920;

    public static volatile int LAUNCH_POS = 2525;
    public static volatile int HANG_POS   = 2850;

    public static volatile double DEFAULT_WORM_POWER     = 1.0;
    public static volatile double DEFAULT_ELEVATOR_POWER = 1.0; // 1.0;
    public static volatile double ELEVATOR_HOMING_POWER  = 0.5; // Absolute value
    public static volatile double WORM_HOMING_POWER      = 0.5; // Absolute value


    // ----------- DRIVE PROPERTIES ---------- //

    public static volatile double DEAD_ZONE_LOW  = 0.9;
    public static volatile double DEAD_ZONE_HIGH = 0.9;
    public static volatile double STRAFE_OFFSET  = 1.1;


    // ----------- INTAKE PROPERTIES ---------- //

    public static volatile double DEFAULT_INTAKE_POWER       = 1.0;
    public static volatile double DEFAULT_OUTTAKE_POWER      = 0.3;
    public static volatile double DEFAULT_INTAKE_BELT_POWER  = 1.0;
    public static volatile double DEFAULT_OUTTAKE_BELT_POWER = 1.0;

    // --------- DELIVERY TRAY PROPERTIES ---------- //

    public static volatile double TRAY_DOOR_OPEN_POS   = 0.1;
    public static volatile double TRAY_DOOR_CLOSED_POS = 0.0;

    // ---------- LAUNCHER PROPERTIES ---------- //

    public static volatile double LAUNCHER_SERVO_ZERO_POS   = 0.00;
    public static volatile double LAUNCHER_SERVO_LAUNCH_POS = 0.60;

    // ---------- HANGER PROPERTIES ---------- //

    public static volatile double HANGER_SERVO_POSITION = 1.0;

    // ----------- CAMERA PROPERTIES ------------ //

    public static volatile int CAMERA_WIDTH = 320;

    public static volatile double LEFT_X  = 0.25 * (double) CAMERA_WIDTH;
    public static volatile double RIGHT_X = 0.75 * (double) CAMERA_WIDTH;
}
