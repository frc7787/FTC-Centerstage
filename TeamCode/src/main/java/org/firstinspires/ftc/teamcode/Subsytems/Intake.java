package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Properties.INTAKE_POWER;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private final DcMotorImplEx leftIntake, rightIntake;

    public Intake(@NonNull HardwareMap hardwareMap) {
        leftIntake  = hardwareMap.get(DcMotorImplEx.class, "LeftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotorImplEx.class, "RightIntakeMotor");
    }

    /**
     * Initializes the motor subsystem.
     * This sets the zero power behavior of the intake motor to brake
     */
    public void init() {
        leftIntake.setZeroPowerBehavior(FLOAT);
        rightIntake.setZeroPowerBehavior(FLOAT);

        leftIntake.setDirection(REVERSE);
    }

    /**
     * Spins the intake
     */
    public void intake() {
        leftIntake.setPower(INTAKE_POWER);
        rightIntake.setPower(INTAKE_POWER);
    }

    /**
     * Displays debug information for the intake
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Intake Debug");

        telemetry.addData("Left Motor Direction", leftIntake.getDirection());
        telemetry.addData("Right Motor Direction", rightIntake.getDirection());

        telemetry.addData("Left Motor Power", leftIntake.getPower());
        telemetry.addData("Right Motor Power", rightIntake.getPower());

        telemetry.addData("Left Motor Current", leftIntake.getCurrent(AMPS));
        telemetry.addData("Right Motor Current", rightIntake.getCurrent(AMPS));
    }
}
