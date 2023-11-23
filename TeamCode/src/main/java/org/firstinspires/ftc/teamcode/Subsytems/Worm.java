package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.TeleOp.Utility.MotorUtility;

final class Worm {

    private final double DEFAULT_POWER = 0.8;
    public final DcMotorImplEx leftWorm, rightWorm;
    public final TouchSensor rotLimitSwitch;
    public final DcMotorImplEx[] wormMotors;

    public Worm(@NonNull HardwareMap hardwareMap) {
        leftWorm       = hardwareMap.get(DcMotorImplEx.class, "lWorm");
        rightWorm      = hardwareMap.get(DcMotorImplEx.class, "rWorm");
        rotLimitSwitch = hardwareMap.get(TouchSensor.class, "lmS2");

        wormMotors = new DcMotorImplEx[]{leftWorm, rightWorm};
    }

    public void checkLimitSwitch() {
        if (leftWorm.getCurrentPosition() == 0 && rotLimitSwitch.isPressed()) {
            MotorUtility.setPower(0, wormMotors);
            MotorUtility.setMode(STOP_AND_RESET_ENCODER, wormMotors);
        }
    }

    public void init() { MotorUtility.setMode(STOP_AND_RESET_ENCODER, wormMotors); }

    public void rotate(int position) { rotate(position, DEFAULT_POWER); }

    public boolean limitSwitchIsPressed() { return rotLimitSwitch.isPressed(); }

    public void rotate(int position, double power) {
        MotorUtility.setTargetPosition(position, wormMotors);
        MotorUtility.setMode(RUN_TO_POSITION, wormMotors);
        MotorUtility.setPower(power, wormMotors);
    }

    public void power(double power) { MotorUtility.setPower(power, wormMotors); }

    public void stop() { MotorUtility.setPower(0, wormMotors); }
}
