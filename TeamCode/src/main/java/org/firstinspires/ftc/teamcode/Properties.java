package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Config
public class Properties {

    // ---------- ARM CONSTANTS ---------- //

    public static volatile int BOTTOM_EXT_POS = 487;
    public static volatile int BOTTOM_ROT_POS = 753;

    public static volatile int LOW_EXT_POS = 850;
    public static volatile int LOW_ROT_POS = 926;

    public static volatile int MED_EXT_POS = 1303;
    public static volatile int MED_ROT_POS = 1365;

    public static volatile int HIGH_EXT_POS = 2011;
    public static volatile int HIGH_ROT_POS = 1684;

    public static volatile int TOP_EXT_POS = 3002;
    public static volatile int TOP_ROT_POS = 1980;

    public static volatile int LAUNCH_POS = 2000;
    public static volatile int HANG_POS   = 2850;
    public static volatile int HUNG_POS   = 0;

    public static volatile double DEFAULT_WORM_POWER     = 1.0;
    public static volatile double DEFAULT_ELEVATOR_POWER = 0.5; //1.0;
    public static volatile double ELEVATOR_HOMING_POWER       = 0.2;//absolute value
    public static volatile double WORM_HOMING_POWER           = 0.2;//absolute value


    // ----------- DRIVE CONSTANTS ---------- //

    public static volatile double DEAD_ZONE_LOW  = 0.9;
    public static volatile double DEAD_ZONE_HIGH = 0.9;
    public static volatile double STRAFE_OFFSET  = 1.1;


    // ----------- INTAKE CONSTANTS ---------- //

    public static volatile double DEFAULT_INTAKE_POWER       = 1.0;
    public static volatile double DEFAULT_OUTTAKE_POWER      = 1.0;
    public static volatile double DEFAULT_INTAKE_BELT_POWER  = 1.0;
    public static volatile double DEFAULT_OUTTAKE_BELT_POWER = 1.0;

    // --------- DELIVERY TRAY CONSTANTS ---------- //

    public static volatile double TRAY_DOOR_OPEN_POS = 0.1;
    public static volatile double TRAY_DOOR_INTAKE_POS = 0.26;
    public static volatile double TRAY_DOOR_CLOSED_POS = 0.0;
    public static volatile double TRAY_UP_WRIST_POS = 1.0;

    // ---------- LAUNCHER CONSTANTS ---------- //

    public static volatile double LAUNCHER_ZERO_POS = 0.00;
    public static volatile double LAUNCHER_SERVO_LAUNCH_POS = 0.60;

    // ---------- HANGER CONSTANTS ---------- //

    public static volatile double HANGER_SERVO_POSITION = 1.0;

    // ---------- OTHER CONSTANTS ------------ //

    public static volatile double ENDGAME_TRIGGER_SENSITIVITY      = 0.9;
    public static volatile double PLANE_LAUNCH_TRIGGER_SENSITIVITY = 0.9;

    // ----------- Camera Constants ------------ //

    public static volatile int CAMERA_WIDTH       = 320;
    public static volatile int ERODE_ITERATIONS   = 7;
    public static volatile int DIALATE_ITERATIONS = 11;

    public static volatile double LEFT_X  = 0.25 * (double) CAMERA_WIDTH;
    public static volatile double RIGHT_X = 0.75 * (double) CAMERA_WIDTH;



    // Darker prop
    //public static volatile Scalar LOW_HSV_RANGE_BLUE = new Scalar(101, 185, 37);
    //public static volatile Scalar HIGH_HSV_RANGE_BLUE = new Scalar(127, 255, 110);


    // -------- Position Constants ----------- //

    public static final Vector2d SHORT_PIXEL_STACK_RED   = new Vector2d(-60.00, -36.00);
    public static final Vector2d LONG_PIXEL_STACK_RED    = new Vector2d(-60.00, -11.60);
    public static final Vector2d BACKDROP_CENTER_POS_RED = new Vector2d(35, -36.70);

    public static final Vector2d SHORT_PIXEL_STACK_BLUE = new Vector2d(-60.00, 36.00);
    public static final Vector2d LONG_PIXEL_STACK_BLUE  = new Vector2d(-60.00, 11.25);
    public static final Vector2d BACKDROP_CENTER_POS_BLUE = new Vector2d(35, 36.00);
}
