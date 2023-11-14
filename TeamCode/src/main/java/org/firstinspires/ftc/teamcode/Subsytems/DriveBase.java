package org.firstinspires.ftc.teamcode.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.Constants.*;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.MotorUtility;

public final class DriveBase {

    private double motorPowerRatio;
    private double drive, strafe, turn;
    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    private final Gamepad controller;
    private final Telemetry telemetry;

    private final DcMotorImplEx frontLeft, frontRight, backLeft, backRight;

    /**
     * Drive Base Subsystem Constructor
     * @param opMode The opMode you are using the drive base in, likely "this"
     */
    public DriveBase(@NonNull OpMode opMode) {
        telemetry  = opMode.telemetry;
        controller = opMode.gamepad1;

        frontLeft  = opMode.hardwareMap.get(DcMotorImplEx.class, "Front Left Drive Motor");
        frontRight = opMode.hardwareMap.get(DcMotorImplEx.class, "Front Right Drive Motor");
        backLeft   = opMode.hardwareMap.get(DcMotorImplEx.class, "Back Left Drive Motor");
        backRight  = opMode.hardwareMap.get(DcMotorImplEx.class, "Back Right Drive Motor");

        MotorUtility.setZeroPowerBehaviour(BRAKE, frontLeft, frontRight, backLeft, backRight);
        MotorUtility.setDirection(REVERSE, frontLeft, backLeft);
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
    public void run() {
        drive  = deadZone(controller.left_stick_y) * -1;
        strafe = deadZone(controller.left_stick_x);
        turn   = deadZone(controller.right_stick_x);

        motorPowerRatio = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        frontLeftPower  = (drive + strafe + turn) / motorPowerRatio;
        frontRightPower = (drive - strafe - turn) / motorPowerRatio;
        backLeftPower   = (drive - strafe + turn) / motorPowerRatio;
        backRightPower  = (drive + strafe - turn) / motorPowerRatio;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }


    /**
     * Provides various debug information about the drive base
     */
    public void debug() {
        telemetry.addLine("Drive Base Debug\n");
        telemetry.addData("Drive Power", drive);
        telemetry.addData("Strafe Power", strafe);
        telemetry.addData("Turn Power", turn);
        telemetry.addData("Motor Power Ratio", motorPowerRatio);

        telemetry.addLine("\nDrive Base Motor Powers\n");

        telemetry.addData("Front Left Drive Motor Power", frontLeftPower);
        telemetry.addData("Front Right Drive Motor Power", frontRightPower);
        telemetry.addData("Back Left Drive Motor Power", backLeftPower);
        telemetry.addData("Back Right Drive Motor Power", backRightPower);

        telemetry.update();
    }
}
