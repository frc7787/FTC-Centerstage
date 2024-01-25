package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp(name = "Hopefully pls make arm move")
public class MoveWorm extends OpMode {

    DcMotorImplEx extensionMotor;

    @Override public void init() {
        extensionMotor = hardwareMap.get(DcMotorImplEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override public void loop() {
        telemetry.addData("Extension Motor Current Pos", extensionMotor.getCurrentPosition());
        telemetry.addData("Extension Motor Enabled", extensionMotor.isMotorEnabled());

        extensionMotor.setPower(gamepad1.left_stick_y * -1);

        telemetry.update();
    }
}
