package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_HIGH;
import static org.firstinspires.ftc.teamcode.Properties.DEAD_ZONE_LOW;
import static org.firstinspires.ftc.teamcode.Properties.STRAFE_OFFSET;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utility.MotorUtility;

public class DriveBase {

    private double motorPowerRatio, fLPower, fRPower, bLPower, bRPower;

    private final DcMotorImplEx fL, fR, bL, bR;
    private final DcMotorImplEx[] motors;

    public DriveBase(@NonNull HardwareMap hardwareMap) {
        fL  = hardwareMap.get(DcMotorImplEx.class, "FrontLeftDriveMotor");
        fR  = hardwareMap.get(DcMotorImplEx.class, "FrontRightDriveMotor");
        bL  = hardwareMap.get(DcMotorImplEx.class, "BackLeftDriveMotor");
        bR  =  hardwareMap.get(DcMotorImplEx.class, "BackRightDriveMotor");

        motors = new DcMotorImplEx[]{fL, fR, bL, bR};
    }

    public void init() {
        MotorUtility.setZeroPowerBehaviour(BRAKE, motors);
        MotorUtility.setDirection(REVERSE, fL, bL);
    }


    private double deadZone(double value) {
        if (DEAD_ZONE_LOW < value && DEAD_ZONE_HIGH > value ) { return 0.0d; }
        return value;
    }

    /**
     * Function to test the drive base
     * @param strafe The strafe power to apply
     * @param drive The drive power to apply
     * @param turn The turn power to apply
     */
    public void drive(double strafe, double drive, double turn) {
        drive  = deadZone(drive)  * -1;
        strafe = deadZone(strafe) * STRAFE_OFFSET;
        turn   = deadZone(turn);

        motorPowerRatio = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        fLPower = (drive + strafe + turn) / motorPowerRatio;
        fRPower = (drive - strafe - turn) / motorPowerRatio;
        bLPower = (drive - strafe + turn) / motorPowerRatio;
        bRPower = (drive + strafe - turn) / motorPowerRatio;

        fL.setPower(fLPower);
        fR.setPower(fRPower);
        bL.setPower(bLPower);
        bR.setPower(bRPower);
    }

    public void stop() { MotorUtility.setPower(0, motors); }
    /**
     * Provides various debug information about the drive base
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Drive Base Debug\n");

        telemetry.addData("Motor Power Ratio", motorPowerRatio);

        telemetry.addData("Front Left Drive Motor Power", fLPower);
        telemetry.addData("Front Right Drive Motor Power", fRPower);
        telemetry.addData("Back Left Drive Motor Power", bLPower);
        telemetry.addData("Back Right Drive Motor Power", bRPower);

        telemetry.addData("Front Left Drive Motor Direction", fL.getDirection());
        telemetry.addData("Front Right Drive Motor Direction", fR.getDirection());
        telemetry.addData("Back Left Drive Motor Direction", bL.getDirection());
        telemetry.addData("Back Right Drive Motor Direction", bR.getDirection());

        telemetry.addData("Front Left Drive Motor Current (AMPS)", fL.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front Right Drive Motor Current (AMPS)", fR.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Back Left Drive Motor Current (AMPS)", bL.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Back Right Drive Motor Current (AMPS)", bR.getCurrent(CurrentUnit.AMPS));
    }
}
