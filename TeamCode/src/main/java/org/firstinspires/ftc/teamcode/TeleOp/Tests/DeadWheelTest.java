package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Dead Wheel Test", group = "Test")
public class DeadWheelTest extends OpMode {

    private DcMotorImplEx left, right, front;

    @Override public void init() {
        left  = hardwareMap.get(DcMotorImplEx.class, "FrontLeftDriveMotor");
        right = hardwareMap.get(DcMotorImplEx.class, "FrontRightDriveMotor");
        front = hardwareMap.get(DcMotorImplEx.class, "IntakeMotor");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override public void loop() {
        telemetry.addData("Left Encoder Position", left.getCurrentPosition());
        telemetry.addData("Right Encoder Position", right.getCurrentPosition());
        telemetry.addData("Front Encoder Position", front.getCurrentPosition());
    }
}
