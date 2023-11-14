package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class DriveConstants {

    /* Motor Stats That Should Be Found Online */
    public static final double TICKS_PER_REV = 537;
    public static final double MAX_RPM = 312;


    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    // Units are in inches
    public static double WHEEL_RADIUS = 1.88d;
    public static double GEAR_RATIO   = 1.00d;
    public static double TRACK_WIDTH  = 13.8d;

    /* FeedForward PID Parameters */
    public static double kV      = 0.016d;
    public static double kA      = 0.0022d;
    public static double kStatic = 0; // This value isn't implemented properly and shouldn't be touched

    /* Trajectory Values */
    public static double THEORETICAL_MAX_VEL = 61.743;
    public static double MAX_VEL             = THEORETICAL_MAX_VEL / 2;
    public static double MAX_ACCEL           = MAX_VEL * 2;
    public static double MAX_ANG_VEL         = 344.315;
    public static double MAX_ANG_ACCEL       = Math.toRadians(60);

    /*
     * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
     */
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

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
