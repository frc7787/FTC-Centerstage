package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

import org.firstinspires.ftc.teamcode.TeleOp.Utility.MotorUtility;

@TeleOp(name = "Test - Dead Wheels", group = "Test")
public final class DeadWheelTest extends OpMode {

    // Dead wheel names
    public static DcMotorEx left, right, strafe;

    @Override public void init() {
        left   = hardwareMap.get(DcMotorEx.class, "Front Left Drive Motor");
        right  = hardwareMap.get(DcMotorEx.class, "Front Right Drive Motor");
        strafe = hardwareMap.get(DcMotorEx.class, "Back Left Drive Motor");

        MotorUtility.setMode(STOP_AND_RESET_ENCODER, left, right, strafe);
    }

    @Override public void loop() {
        telemetry.addData("Left Dead Wheel Position", left.getCurrentPosition());
        telemetry.addData("Right Dead Wheel Position", right.getCurrentPosition());
        telemetry.addData("Strafe Dead Wheel Position", strafe.getCurrentPosition());
    }
}
