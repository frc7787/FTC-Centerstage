package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test - Worm", group = "Test")
public class WormTest extends OpMode {

    public DcMotorEx lWorm, rWorm, lExtend, rExtend;
    public Servo wrist;

    @Override
    public void init() {
        lWorm = hardwareMap.get(DcMotorImplEx.class, "lWorm");
        rWorm = hardwareMap.get(DcMotorImplEx.class, "rWorm");

        lExtend = hardwareMap.get(DcMotorEx.class, "lExt");
        rExtend = hardwareMap.get(DcMotorEx.class, "rExt");

        wrist = hardwareMap.get(Servo.class, "wristServo");

        lWorm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rWorm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lWorm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rWorm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist.scaleRange(0.1, 0.9);
        wrist.setPosition(0.1);
    }

    @Override
    public void loop() {
        wrist.setPosition(gamepad1.left_trigger);

        telemetry.addData("Rotation Position", lWorm.getCurrentPosition());

        telemetry.addData("Left Worm Position", lWorm.getCurrentPosition());
        telemetry.addData("Right Worm Position", rWorm.getCurrentPosition());

        telemetry.addData("Extend Position", lExtend.getCurrentPosition());

        telemetry.addData("Wrist Position", wrist.getPosition());

        telemetry.update();
    }
}
