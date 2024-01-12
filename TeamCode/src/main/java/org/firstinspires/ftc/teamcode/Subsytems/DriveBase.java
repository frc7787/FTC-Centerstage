package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_HIGH;
import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_LOW;
import static org.firstinspires.ftc.teamcode.Properties.STRAFE_OFFSET;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility.MotorUtility;

public class DriveBase {

    IMU imu;

    DcMotorImplEx fL, fR, bL, bR;

    DcMotorImplEx[] motors;

    public DriveBase(@NonNull HardwareMap hardwareMap) {
        fL = hardwareMap.get(DcMotorImplEx.class, "FrontLeftDriveMotor");
        fR = hardwareMap.get(DcMotorImplEx.class, "FrontRightDriveMotor");
        bL = hardwareMap.get(DcMotorImplEx.class, "BackLeftDriveMotor");
        bR = hardwareMap.get(DcMotorImplEx.class, "BackRightDriveMotor");

        imu = hardwareMap.get(IMU.class, "imu");

        motors = new DcMotorImplEx[]{fL, fR, bL, bR};
    }

    /**
     * Initializes the DriveBase Class.
     * Reverses the left motors (fL, BL) and sets all motor zero power behaviours to brake
     */
    public void init() {
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        MotorUtility.setDirection(REVERSE, fL, bL);
        MotorUtility.setZeroPowerBehaviour(BRAKE, motors);
    }

    /**
     * Returns the value if is outise of the range defined by the DEAD_ZONE_LOW and DEAD_ZONE_HIGH values.
     * If the value is in range, returns 0.0d
     * @param value: The value to check
     */
    private double deadZone(double value) {
        if (DEAD_ZONE_LOW < value && DEAD_ZONE_HIGH > value ) { return 0.0d; }
        return value;
    }

    /**
     * Drives the robot relative to the field
     * 
     * @param drive: The forward translational value
     * @param strafe: The side to side translational value
     * @param turn: THe rotational value
     */
    public void driveManualFieldCentric(double drive, double strafe, double turn) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        drive  = deadZone(drive);
        strafe = deadZone(strafe);

        drive  = drive * Math.cos(-botHeading) - strafe * Math.sin(-botHeading);
        strafe = drive  * Math.sin(-botHeading) + strafe * Math.cos(-botHeading);

        double motorPowerRatio = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double fLPower = (drive + strafe + turn) / motorPowerRatio;
        double fRPower = (drive - strafe - turn) / motorPowerRatio;
        double bLPower = (drive - strafe + turn) / motorPowerRatio;
        double bRPower = (drive + strafe - turn) / motorPowerRatio;

        fL.setPower(fLPower);
        fR.setPower(fRPower);
        bL.setPower(bLPower);
        bR.setPower(bRPower);
    }

    /**
     * Drives the robot relative to itself
     * 
     * @param drive: The forward translational value
     * @param strafe: The side to side translational value
     * @param turn: The rotational value
     */
    public void driveManualRobotCentric(double drive, double strafe, double turn) {
        drive  = deadZone(drive);
        strafe = deadZone(strafe) * STRAFE_OFFSET;
        turn   = deadZone(turn);

        double motorPowerRatio = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double fLPower = (drive + strafe + turn) / motorPowerRatio;
        double fRPower = (drive - strafe - turn) / motorPowerRatio;
        double bLPower = (drive - strafe + turn) / motorPowerRatio;
        double bRPower = (drive + strafe - turn) / motorPowerRatio;

        fL.setPower(fLPower);
        fR.setPower(fRPower);
        bL.setPower(bLPower);
        bR.setPower(bRPower);
    }
}
