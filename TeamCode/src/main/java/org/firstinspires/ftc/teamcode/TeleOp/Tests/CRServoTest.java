package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@Autonomous(name = "Test - Intake Belt Servo")
public class CRServoTest extends OpMode {

    CRServoImplEx intakeBeltServo;

    Intake intake;

    @Override public void init() {
        intakeBeltServo = hardwareMap.get(CRServoImplEx.class, "Intake Belt Servo");
        intakeBeltServo.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new Intake(hardwareMap);
    }

    @Override public void loop() {
        intake.intake();
        intakeBeltServo.setPower(1.0);
    }
}
