package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Test - Wrist Servo")
@Disabled
public class TestWristServo extends OpMode {

    ServoImplEx wristServo;

    DcMotorImplEx elevatorMotor;

    private double SERVO_POS = 0.0;

    private boolean leftBumperWasPressed  = false;
    private boolean rightBumperWasPressed = false;

    @Override public void init() {
        // Expansion Hub Port 0
        wristServo = hardwareMap.get(ServoImplEx.class, "WristServo");

        // Control Hub Port 3; Encoder Port 3
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class,  "ExtensionMotor");
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        wristServo.setPosition(0.0);
    }

    @Override public void loop() {
        if (gamepad1.options) { // Reset Position
            SERVO_POS = 0;
        }

        if (gamepad1.left_bumper) { // Increment Position
           if (!leftBumperWasPressed) {
               SERVO_POS += 0.01;

               if (SERVO_POS > 1.0) {
                   SERVO_POS = 1;
               }

               leftBumperWasPressed = true;
           }
        } else {
            leftBumperWasPressed = false;
        }

        if (gamepad1.right_bumper) { // Decrement Position
            if (!rightBumperWasPressed) {
                SERVO_POS -= 0.01;

                if (SERVO_POS < 0.0) {
                    SERVO_POS = 0;
                }

                rightBumperWasPressed = true;
            }
        } else {
            rightBumperWasPressed = false;
        }

        wristServo.setPosition(SERVO_POS);

        telemetry.addData("Wrist Servo Commanded Position", wristServo.getPosition());

        telemetry.update();
    }
}
