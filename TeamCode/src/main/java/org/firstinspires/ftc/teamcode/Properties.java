package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Properties {

    // ---------- ELEVATOR CONSTANTS ---------- //

    public static volatile int BOTTOM_EXT_POS = RobotPropertyParser.getInt("BOTTOM_EXT_POS");
    public static volatile int BOTTOM_ROT_POS = RobotPropertyParser.getInt("BOTTOM_ROT_POS");

    public static volatile int LOW_EXT_POS = RobotPropertyParser.getInt("LOW_EXT_POS");
    public static volatile int LOW_ROT_POS = RobotPropertyParser.getInt("LOW_ROT_POS");

    public static volatile int MED_EXT_POS = RobotPropertyParser.getInt("MED_EXT_POS");
    public static volatile int MED_ROT_POS = RobotPropertyParser.getInt("MED_ROT_POS");

    public static volatile int HIGH_EXT_POS = RobotPropertyParser.getInt("HIGH_EXT_POS");
    public static volatile int HIGH_ROT_POS = RobotPropertyParser.getInt("HIGH_ROT_POS");

    public static volatile int TOP_EXT_POS = RobotPropertyParser.getInt("TOP_EXT_POS");
    public static volatile int TOP_ROT_POS = RobotPropertyParser.getInt("TOP_ROT_POS");

    public static volatile int LAUNCH_POS = RobotPropertyParser.getInt("LAUNCH_POS");
    public static volatile int HANG_POS   = RobotPropertyParser.getInt("HANG_POS");
    public static volatile int HUNG_POS   = RobotPropertyParser.getInt("HUNG_POS");

    public static volatile double DEFAULT_WORM_POWER     = RobotPropertyParser.getDouble("DEFAULT_WORM_POWER");
    public static volatile double DEFAULT_ELEVATOR_POWER = RobotPropertyParser.getDouble("DEFAULT_ELEVATOR_POWER");
    public static volatile double HOMING_POWER           = RobotPropertyParser.getDouble("HOMING_POWER");

    // ----------- DRIVE CONSTANTS ---------- //

    public static volatile double DEAD_ZONE_LOW  = RobotPropertyParser.getDouble("DEAD_ZONE_LOW");
    public static volatile double DEAD_ZONE_HIGH = RobotPropertyParser.getDouble("DEAD_ZONE_HIGH");
    public static volatile double STRAFE_OFFSET  = RobotPropertyParser.getDouble("STRAFE_OFFSET");


    // ----------- INTAKE CONSTANTS ---------- //

    public static volatile double INTAKE_SPEED  = RobotPropertyParser.getDouble("INTAKE_SPEED");
    public static volatile double OUTTAKE_SPEED = RobotPropertyParser.getDouble("OUTTAKE_SPEED");

    // ---------- LAUNCHER CONSTANTS ---------- //

    public static volatile double LAUNCHER_SERVO_POSITION = RobotPropertyParser.getDouble("LAUNCHER_SERVO_POSITION");

    // ---------- HANGER CONSTANTS ---------- //

    public static volatile double HANGER_SERVO_POSITION = RobotPropertyParser.getDouble("HANGER_SERVO_POSITION");

    // ---------- OTHER CONSTANTS ------------ //

    public static volatile double ENDGAME_TRIGGER_SENSITIVITY      = RobotPropertyParser.getDouble("ENDGAME_TRIGGER_SENSITIVITY");
    public static volatile double PLANE_LAUNCH_TRIGGER_SENSITIVITY = RobotPropertyParser.getDouble("PLANE_LAUNCH_TRIGGER_SENSITIVITY");
}
