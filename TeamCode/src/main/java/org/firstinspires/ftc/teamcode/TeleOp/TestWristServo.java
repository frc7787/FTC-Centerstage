package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Test - Wrist Servo")
public class TestWristServo extends OpMode {

    ServoImplEx wristServo;

    DcMotorImplEx elevatorMotor;

    @Override public void init() {
        wristServo    = hardwareMap.get(ServoImplEx.class, "WristServo");

        // Control Hub Port 3; Encoder Port 3
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class,  "ElevatorMotor");
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override public void loop() {
        if (elevatorMotor.getCurrentPosition() > 500) {
            wristServo.setPosition(1);
        } else {
            wristServo.setPosition(0);
        }

        telemetry.addData("Elevator Motor Position", elevatorMotor.getCurrentPosition());
        telemetry.addData("Wrist Servo Commanded Position", wristServo.getPosition());


    }
}
