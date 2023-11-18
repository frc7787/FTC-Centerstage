package org.firstinspires.ftc.teamcode.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.*;

import androidx.annotation.NonNull;

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

    /**
     * Drive Base Subsystem Constructor
     * @param opMode The opMode you are using the drive base in, likely "this"
     */
    public DriveBase(@NonNull HardwareMap hardwareMap) {
        fL  = hardwareMap.get(DcMotorImplEx.class, "fldm");
        fR  = hardwareMap.get(DcMotorImplEx.class, "frdm");
        bL  = hardwareMap.get(DcMotorImplEx.class, "bldm");
        bR  = hardwareMap.get(DcMotorImplEx.class, "brdm");

        MotorUtility.setZeroPowerBehaviour(BRAKE, fL, fR, bL, bR);
        MotorUtility.setDirection(REVERSE, fL, bL);
    }


    /**
     * Checks to see if a value is in a dead zone
     * @param value The value to check
     * @return If The value is in the dead zone returns 0.0d, else returns the input value
     */
    private double deadZone(double value) {
        if (DEAD_ZONE_LOW < value && DEAD_ZONE_HIGH > value ) { return 0.0d; }
        return value;
    }


    /**
     * Main function to control the drive base
     */
    public void run(@NonNull Gamepad controller) {
        drive  = deadZone(controller.left_stick_y) * -1;
        strafe = deadZone(controller.left_stick_x);
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

    public void strafe(StrafeDirections direction, int duration, LinearOpMode opMode) {
        double power = 0.5;
        double negativePower = -0.5;

        if (direction == StrafeDirections.RIGHT) {
            fL.setPower(power);
            fR.setPower(negativePower);
            bL.setPower(negativePower);
            bR.setPower(power);
            opMode.sleep(duration);
        } else if (direction == StrafeDirections.LEFT) {
            fL.setPower(negativePower);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(negativePower);
            opMode.sleep(duration);
        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }


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
