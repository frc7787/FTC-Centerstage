package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Properties {

    // ---------- ELEVATOR CONSTANTS ---------- //

    public static volatile int BOTTOM_EXT_POS = 1890;
    public static volatile int BOTTOM_ROT_POS = 528;

    public static volatile int LOW_EXT_POS = 2041;
    public static volatile int LOW_ROT_POS = 2633;

    public static volatile int MED_EXT_POS = 2421;
    public static volatile int MED_ROT_POS = 804;

    public static volatile int HIGH_EXT_POS = 2983;
    public static volatile int HIGH_ROT_POS = 947;

    public static volatile int TOP_EXT_POS = 3045;
    public static volatile int TOP_ROT_POS = 1107;

    public static volatile int LAUNCH_POS = 1630;
    public static volatile int HANG_POS   = 2326;
    public static volatile int HUNG_POS   = 420;

    public static volatile double DEFAULT_WORM_POWER     = 0.8;
    public static volatile double DEFAULT_ELEVATOR_POWER = 0.9;
    public static volatile double HOMING_POWER           = 0.7;

    // ----------- DRIVE CONSTANTS ---------- //

    public static volatile double DEAD_ZONE_LOW  = 0.9;
    public static volatile double DEAD_ZONE_HIGH = 0.9;
    public static volatile double STRAFE_OFFSET  = 1.1;


    // ----------- INTAKE CONSTANTS ---------- //

    public static volatile double INTAKE_POWER = 1.0;

    // ---------- LAUNCHER CONSTANTS ---------- //

    public static volatile double LAUNCHER_SERVO_POSITION = 1.0;

    // ---------- HANGER CONSTANTS ---------- //

    public static volatile double HANGER_SERVO_POSITION = 1.0;

    // ---------- OTHER CONSTANTS ------------ //

    public static volatile double ENDGAME_TRIGGER_SENSITIVITY      = 0.9;
    public static volatile double PLANE_LAUNCH_TRIGGER_SENSITIVITY = 0.9;
}
