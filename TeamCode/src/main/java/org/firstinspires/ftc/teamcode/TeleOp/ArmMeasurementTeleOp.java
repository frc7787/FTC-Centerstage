package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Utility - Arm Measurement")
public class ArmMeasurementTeleOp extends OpMode {

    DcMotorImplEx wormMotor, elevatorMotor;

    @Override public void init() {
        wormMotor     = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class, "ExtensionMotor");

        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override public void loop() {
        telemetry.addLine("The elevator and worm does not have power.");
        telemetry.addLine("Move the elevator and worm to the desired positions");

        telemetry.addData("Elevator Current Position", elevatorMotor.getCurrentPosition());
        telemetry.addData("Worm Current Position", wormMotor.getCurrentPosition());
        telemetry.update();
    }
}
