package org.firstinspires.ftc.teamcode;

public class Properties {

    // ---------- ELEVATOR CONSTANTS ---------- //

    public static int BOTTOM_EXTEND_POSITION = GetRobotProperties.readInteger("BOTTOM_EXTEND_POSITION");
    public static int BOTTOM_ROT_POSITION    = GetRobotProperties.readInteger("BOTTOM_ROT_POSITION");

    public static int LOW_EXTEND_POSITION = GetRobotProperties.readInteger("LOW_EXTEND_POSITION");
    public static int LOW_ROT_POSITION    = GetRobotProperties.readInteger("LOW_ROT_POSITION");

    public static int MED_EXTEND_POSITION = GetRobotProperties.readInteger("MED_EXTEND_POSITION");
    public static int MED_ROT_POSITION    = GetRobotProperties.readInteger("MED_ROT_POSITION");

    public static int HIGH_EXTEND_POSITION = GetRobotProperties.readInteger("HIGH_EXTEND_POSITION");
    public static int HIGH_ROT_POSITION    = GetRobotProperties.readInteger("HIGH_ROT_POSITION");

    public static int TOP_EXTEND_POSITION = GetRobotProperties.readInteger("TOP_EXTEND_POSITION");
    public static int TOP_ROT_POSITION    = GetRobotProperties.readInteger("TOP_ROT_POSITION");

    public static int LAUNCH_POSITION = GetRobotProperties.readInteger("LAUNCH_POSITION");
    public static int HANG_POSITION   = GetRobotProperties.readInteger("HANG_POSITION");
    public static int HUNG_POSITION   = GetRobotProperties.readInteger("HUNG_POSITION");

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
