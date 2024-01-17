package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class RoadRunnerConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 537.7d;
    public static final double MAX_RPM       = 312.0d;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60.0 * TICKS_PER_REV));

    // Inches
    public static double WHEEL_RADIUS = 1.88976d;
    public static double GEAR_RATIO   = 1.01d;
    public static double TRACK_WIDTH  = 15.84;

    // Feedforward PID values
    public static double kV      = 0.0134d;
    public static double kA      = 0.0d;
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

    /**
     * Converts encoder ticks to inches using the following formula: <br>
     * Wheel Radius * 2 * pi * Gear Ratio * Ticks / Ticks Per Revolution (Motor)
     * @param ticks The ticks to convert into inches
     * @return The inches, converted from ticks
     */
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2.0 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    /**
     * Converts RPM (Revolutions Per Minute) into velocity using the following formula: <br>
     * RPM * Gear Ratio * 2 * Pi * Wheel Radius / 60.0
     * @param rpm The RPM to convert
     * @return The calculated velocity
     */
    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) { return 32767.0 / ticksPerSecond; }
}