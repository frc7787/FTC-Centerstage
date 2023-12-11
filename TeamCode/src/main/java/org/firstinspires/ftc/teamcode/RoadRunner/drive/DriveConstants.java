package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 537.7d;
    public static final double MAX_RPM       = 312.0d;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    // Inches
    public static double WHEEL_RADIUS = 1.85d;
    public static double GEAR_RATIO   = 1.01d;
    public static double TRACK_WIDTH  = 15.84;

    // Feedforward PID values
    public static double kV      = 0.01375;
    public static double kA      = 0.002;
    public static double kStatic = 0.0d;

    // It is best if you don't exceed 80% of your robots abilities; All units are inches and Radians
    public static double MAX_VEL        = 74.3;
    public static double MAX_ACCEL      = 50.0d;
    public static double MAX_ANG_VEL    = 3.6;
    public static double MAX_ANG_ACCEL  = Math.PI;

    // Orientation Adjusting, since we aren't using the IMU we shouldn't need to change
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) { return 32767 / ticksPerSecond;
    }
}