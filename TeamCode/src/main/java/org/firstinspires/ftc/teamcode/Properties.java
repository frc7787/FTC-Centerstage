package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

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

    public static volatile double DEFAULT_WORM_POWER     = 1;
    public static volatile double DEFAULT_ELEVATOR_POWER = 1;
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

    // ----------- Camera Constants ------------ //

    public static volatile int CAMERA_WIDTH       = 340;
    public static volatile int ERODE_ITERATIONS   = 7;
    public static volatile int DIALATE_ITERATIONS = 11;

    public static volatile double LEFT_X  = 0.25 * (double) CAMERA_WIDTH;
    public static volatile double RIGHT_X = 0.75 * (double) CAMERA_WIDTH;

    public static volatile Scalar BOUNDING_RECTANGLE_COLOR = new Scalar(0.5, 76.9, 89.8);

    public static volatile Scalar LOW_HSV_RANGE_BLUE  = new Scalar(97, 100, 100);
    public static volatile Scalar HIGH_HSV_RANGE_BLUE = new Scalar(115, 255, 255);

    public static volatile Scalar LOW_HSV_RANGE_RED_ONE  = new Scalar(160, 150, 0);
    public static volatile Scalar HIGH_HSV_RANGE_RED_ONE = new Scalar(180, 255, 255);

    public static volatile Scalar LOW_HSV_RANGE_RED_TWO   = new Scalar(0, 160, 0);
    public static volatile Scalar HIGH_HSV_RANGLE_RED_TWO = new Scalar(10, 255, 255);

    public static volatile Point CV_ANCHOR        = new Point(-1, -1);
    public static volatile Scalar CV_BORDER_VALUE = new Scalar(-1);
    public static volatile int CV_BORDER_TYPE     = Core.BORDER_CONSTANT;
    public static volatile Rect CROP_RECT         = new Rect(0, 80, 320, 80);
}
