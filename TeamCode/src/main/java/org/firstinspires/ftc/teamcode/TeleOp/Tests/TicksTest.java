package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import dalvik.bytecode.Opcodes;

@TeleOp(name = "Test - Elevator Ticks")
@Disabled
public class TicksTest extends OpMode {

    DcMotorImplEx motor;

    @Override public void init() {
        motor = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override public void loop() {
        telemetry.addData("Worm Ticks", motor.getCurrentPosition());
        telemetry.update();
    }
}
