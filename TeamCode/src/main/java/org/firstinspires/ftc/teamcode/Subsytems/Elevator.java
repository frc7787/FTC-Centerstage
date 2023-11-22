package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Utility.MotorUtility;

/**
 * Object to encapsulate elevator subsystem
 */
final class Elevator {

    private final DcMotorImplEx leftExtend, rightExtend;
    private final DcMotorImplEx[] elevatorMotors;
    private final TouchSensor extLimitSwitch;


    public Elevator(@NonNull HardwareMap hardwareMap) {
        leftExtend     = hardwareMap.get(DcMotorImplEx.class, "lExt");
        rightExtend    = hardwareMap.get(DcMotorImplEx.class, "rExt");
        extLimitSwitch = hardwareMap.get(TouchSensor.class, "lmS");
        elevatorMotors = new DcMotorImplEx[]{leftExtend, rightExtend};
    }


    public void init() {
        MotorUtility.setMode(STOP_AND_RESET_ENCODER, elevatorMotors);
        rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void checkLimitSwitch() {
        if (leftExtend.getCurrentPosition() == 0 && extLimitSwitch.isPressed()) {
            leftExtend.setPower(0);
            rightExtend.setPower(0);

            leftExtend.setMode(STOP_AND_RESET_ENCODER);
            rightExtend.setMode(STOP_AND_RESET_ENCODER);
        }
    }

    public void extend(int position) {
        MotorUtility.setTargetPosition(position, elevatorMotors);
        MotorUtility.setMode(RUN_TO_POSITION, elevatorMotors);
        MotorUtility.setPower(0.9, elevatorMotors);
    }


    public void power(double power) { MotorUtility.setPower(power, elevatorMotors); }

    public boolean limitSwitchIsPressed() { return extLimitSwitch.isPressed(); }


    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Elevator Debug");

        telemetry.addData("Target Position", leftExtend.getCurrentPosition());
        telemetry.addData("Left Motor Current Position", leftExtend.getCurrentPosition());
        telemetry.addData("Right Motor Current Position", rightExtend.getCurrentPosition());

        telemetry.addData("LS extension", extLimitSwitch.isPressed());

        telemetry.update();
    }

    public void stop() { MotorUtility.setPower(0, elevatorMotors); }
}

