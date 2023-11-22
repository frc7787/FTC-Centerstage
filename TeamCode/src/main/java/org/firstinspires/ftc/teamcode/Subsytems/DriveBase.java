package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Constants.DEAD_ZONE_HIGH;
import static org.firstinspires.ftc.teamcode.Constants.DEAD_ZONE_LOW;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Utility.MotorUtility;

public final class DriveBase {

    public enum StrafeDirections {
        RIGHT,
        LEFT
    }

    private double motorPowerRatio;
    private double drive, strafe, turn;
    private double fLPower, fRPower, bLPower, bRPower;

    private final DcMotorImplEx fL, fR, bL, bR;
    private final DcMotorImplEx[] motors;

    public DriveBase(@NonNull HardwareMap hardwareMap) {
        fL  = hardwareMap.get(DcMotorImplEx.class, "fldm");
        fR  = hardwareMap.get(DcMotorImplEx.class, "frdm");
        bL  = hardwareMap.get(DcMotorImplEx.class, "bldm");
        bR  =  hardwareMap.get(DcMotorImplEx.class, "brdm");

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
     * Main function to control the drive base
     */
    public void run(@NonNull Gamepad controller) {
        drive  = deadZone(controller.left_stick_y) * -1;
        strafe = deadZone(controller.left_stick_x) * 1.1;
        turn   = deadZone(controller.right_stick_x);

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

    public void strafe(@NonNull StrafeDirections direction) {
        double power = 0.5;

        if (direction == StrafeDirections.RIGHT) {
            MotorUtility.setPower(power, fL, bR);
            MotorUtility.setPower(-power, fR, bL);

        } else if (direction == StrafeDirections.LEFT) {
            MotorUtility.setPower(power, fR, bL);
            MotorUtility.setPower(-power, fL, bR);
        }

        MotorUtility.setPower(0, motors);
    }

    public void driveBackwards() {
        double power = -0.5;
        MotorUtility.setPower(power, motors);
    }

    public void stop() { MotorUtility.setPower(0, motors); }


    /**
     * Provides various debug information about the drive base
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Drive Base Debug\n");

        telemetry.addData("Drive Power", drive);
        telemetry.addData("Strafe Power", strafe);
        telemetry.addData("Turn Power", turn);
        telemetry.addData("Motor Power Ratio", motorPowerRatio);
        telemetry.addData("Front Left Drive Motor Power", fLPower);
        telemetry.addData("Front Right Drive Motor Power", fRPower);
        telemetry.addData("Back Left Drive Motor Power", bLPower);
        telemetry.addData("Back Right Drive Motor Power", bRPower);
    }
}
