package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_HIGH;
import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_LOW;
import static org.firstinspires.ftc.teamcode.Properties.STRAFE_OFFSET;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveBase {

    DcMotorEx fL, fR, bL, bR;

    public DriveBase(HardwareMap hardwareMap) {
        fL = hardwareMap.get(DcMotorEx.class, "FrontLeftDriveMotor");
        fR = hardwareMap.get(DcMotorEx.class, "FrontRightDriveMotor");
        bL = hardwareMap.get(DcMotorEx.class, "BackLeftDriveMotor");
        bR = hardwareMap.get(DcMotorEx.class, "BackRightDriveMotor");
    }

    public void init() {
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double deadZone(double value) {
        if (DEAD_ZONE_LOW < value && DEAD_ZONE_HIGH > value ) { return 0.0; }
        return value;
    }

    public void driveManual(double drive, double strafe, double turn) {
        drive  = deadZone(drive)  * -1.0;
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
