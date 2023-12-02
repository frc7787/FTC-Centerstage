package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Properties {

    // ---------- ELEVATOR CONSTANTS ---------- //

    public static int BOTTOM_EXT_POS = GetRobotProperties.readInteger("BOTTOM_EXT_POS");
    public static int BOTTOM_ROT_POS = GetRobotProperties.readInteger("BOTTOM_ROT_POS");

    public static int LOW_EXT_POS = GetRobotProperties.readInteger("LOW_EXT_POS");
    public static int LOW_ROT_POS = GetRobotProperties.readInteger("LOW_ROT_POS");

    public static int MED_EXT_POS = GetRobotProperties.readInteger("MED_EXT_POS");
    public static int MED_ROT_POS = GetRobotProperties.readInteger("MED_ROT_POS");

    public static int HIGH_EXT_POS = GetRobotProperties.readInteger("HIGH_EXT_POS");
    public static int HIGH_ROT_POS = GetRobotProperties.readInteger("HIGH_ROT_POS");

    public static int TOP_EXT_POS = GetRobotProperties.readInteger("TOP_EXT_POS");
    public static int TOP_ROT_POS = GetRobotProperties.readInteger("TOP_ROT_POS");

    public static int LAUNCH_POS = GetRobotProperties.readInteger("LAUNCH_POS");
    public static int HANG_POS   = GetRobotProperties.readInteger("HANG_POS");
    public static int HUNG_POS   = GetRobotProperties.readInteger("HUNG_POS");

    public static double DEFAULT_WORM_POWER     = GetRobotProperties.readDouble("DEFAULT_WORM_POWER");
    public static double DEFAULT_ELEVATOR_POWER = GetRobotProperties.readDouble("DEFAULT_ELEVATOR_POWER");
    public static double HOMING_POWER           = GetRobotProperties.readDouble("HOMING_POWER");

    // ----------- DRIVE CONSTANTS ---------- //

    public static double DEAD_ZONE_LOW  = GetRobotProperties.readDouble("DEAD_ZONE_LOW");
    public static double DEAD_ZONE_HIGH = GetRobotProperties.readDouble("DEAD_ZONE_HIGH");
    public static double STRAFE_OFFSET  = GetRobotProperties.readDouble("STRAFE_OFFSET");


    // ----------- INTAKE CONSTANTS ---------- //

    public static double INTAKE_SPEED  = GetRobotProperties.readDouble("INTAKE_SPEED");
    public static double OUTTAKE_SPEED = GetRobotProperties.readDouble("OUTTAKE_SPEED");

    // ---------- LAUNCHER CONSTANTS ---------- //

    public static double LAUNCHER_SERVO_POSITION = GetRobotProperties.readDouble("LAUNCHER_SERVO_POSITION");

    // ---------- HANGER CONSTANTS ---------- //

    public static double HANGER_SERVO_POSITION = GetRobotProperties.readDouble("HANGER_SERVO_POSITION");

    // ---------- OTHER CONSTANTS ------------ //

    public static double ENDGAME_TRIGGER_SENSITIVITY      = GetRobotProperties.readDouble("ENDGAME_TRIGGER_SENSITIVITY");
    public static double PLANE_LAUNCH_TRIGGER_SENSITIVITY = GetRobotProperties.readDouble("PLANE_LAUNCH_TRIGGER_SENSITIVITY");
}
