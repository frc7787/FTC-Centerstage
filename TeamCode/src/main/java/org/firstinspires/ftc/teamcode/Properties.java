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

    public static volatile int BOTTOM_EXT_POS = 2787;
    public static volatile int BOTTOM_ROT_POS = 753;

    public static volatile int LOW_EXT_POS = 3050;
    public static volatile int LOW_ROT_POS = 926;

    public static volatile int MED_EXT_POS = 3003;
    public static volatile int MED_ROT_POS = 1365;

    public static volatile int HIGH_EXT_POS = 3111;
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
    public static volatile double DELIVERY_TRAY_UP_POS = 1.0;

    // ---------- LAUNCHER CONSTANTS ---------- //

    public static volatile double LAUNCHER_ZERO_POS = 0.00;
    public static volatile double LAUNCHER_SERVO_LAUNCH_POS = 0.60;

    // ---------- HANGER CONSTANTS ---------- //

    public static volatile double HANGER_SERVO_POSITION = 1.0;


    // ----------- Camera Constants ------------ //

    public static volatile int CAMERA_WIDTH       = 320;
    public static volatile int ERODE_ITERATIONS   = 7;
    public static volatile int DIALATE_ITERATIONS = 11;

    public static volatile double LEFT_X  = 0.25 * (double) CAMERA_WIDTH;
    public static volatile double RIGHT_X = 0.75 * (double) CAMERA_WIDTH;

    public static volatile Scalar BOUNDING_RECTANGLE_COLOR = new Scalar(255, 0, 0);

    // lighter prop
    public static volatile Scalar LOW_HSV_RANGE_BLUE  = new Scalar(97, 100, 100);
    public static volatile Scalar HIGH_HSV_RANGE_BLUE = new Scalar(115, 255, 255);

    // Darker prop
    //public static volatile Scalar LOW_HSV_RANGE_BLUE = new Scalar(101, 185, 37);
    //public static volatile Scalar HIGH_HSV_RANGE_BLUE = new Scalar(127, 255, 110);

    public static volatile Scalar LOW_HSV_RANGE_RED_ONE  = new Scalar(160, 100, 0);
    public static volatile Scalar HIGH_HSV_RANGE_RED_ONE = new Scalar(180, 255, 255);

    public static volatile Scalar LOW_HSV_RANGE_RED_TWO  = new Scalar(0, 100, 0);
    public static volatile Scalar HIGH_HSV_RANGE_RED_TWO = new Scalar(10, 255, 255);

    public static volatile Point CV_ANCHOR        = new Point(-1, -1);
    public static volatile Scalar CV_BORDER_VALUE = new Scalar(-1);
    public static volatile int CV_BORDER_TYPE     = Core.BORDER_CONSTANT;
    public static volatile Rect CROP_RECT         = new Rect(130, 120, 190, 120);

    // -------- Position Constants ----------- //

    public static final Vector2d SHORT_PIXEL_STACK_RED   = new Vector2d(-60.00, -36.00);
    public static final Vector2d LONG_PIXEL_STACK_RED    = new Vector2d(-60.00, -11.60);
    public static final Vector2d BACKDROP_CENTER_POS_RED = new Vector2d(35, -36.70);

    public static final Vector2d SHORT_PIXEL_STACK_BLUE = new Vector2d(-60.00, 36.00);
    public static final Vector2d LONG_PIXEL_STACK_BLUE  = new Vector2d(-60.00, 11.25);
    public static final Vector2d BACKDROP_CENTER_POS_BLUE = new Vector2d(35, 36.00);
}
