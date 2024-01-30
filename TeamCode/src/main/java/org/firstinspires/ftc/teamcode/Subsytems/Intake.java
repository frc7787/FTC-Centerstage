package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_OUTTAKE_POWER;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.sql.SQLSyntaxErrorException;

/**
 * Class to contain intake subsystem. Not to be confused
 * with the DeliveryTray subsystem.
 */
public class Intake {
    final DcMotorImplEx intake;

    /**
     * Creates a new instance of the intake subsystem
     * @param hardwareMap The hardware map, likely "hardwareMap"
     */
    public Intake(@NonNull HardwareMap hardwareMap) {
       intake = hardwareMap.get(DcMotorImplEx.class, "IntakeMotor");
    }

    /**
     * Initializes the motor subsystem.
     * This sets the zero power behavior of the intake motor to float
     */
    public void init() {
        intake.setDirection(REVERSE);
        intake.setZeroPowerBehavior(FLOAT);
    }

    /**
     * Spins the intake in the intake direction (FORWARD) by the power defined by DEFAULT_INTAKE_POWER
     */
    public void intake() {
        intake(DEFAULT_INTAKE_POWER);
    }

    /**
     * Spins the intake in the intake direction (FORWARD) at the supplied power
     * @param power The power to give the intake motor
     */
    public void intake(double power) {
        intake.setDirection(FORWARD);
        intake.setPower(power);
    }

    /**
     * Spins the intake in the intake direction (FORWARD) for the specified amount of time
     * @param durationMills The time to spin the intake for in milliseconds
     * @param power The power to spin the intake at
     */
    public void intakeForDuration(long durationMills, double power) {
        long start = System.currentTimeMillis();

        while (start + durationMills < System.currentTimeMillis()) {
            intake(power);
        }
    }

    /**
     * Spins the intake in the intake direction (FORWARD) for the specified amount of time
     * @param durationMills The time to spin the intake for in milliseconds
     */
    public void intakeForDuration(long durationMills) {
       intakeForDuration(durationMills, DEFAULT_INTAKE_POWER);
    }

    /**
     * Spins the intake in the outtake direction (REVERSE) at the specified power
     *
     * @param power The power to spin the intake at
     */
    public void outtake(double power) {
        intake.setDirection(REVERSE);
        intake.setPower(power);
    }

    /**
     * Spins the intake in the outtake direction (REVERSE) at the speed specified by
     * DEFAULT_OUTTAKE_POWER.
     */
    public void outtake() {
        outtake(DEFAULT_OUTTAKE_POWER);
    }

    /**
     * Spins the intake in the outtake direction (REVERSE) for the specified amount of time.
     * @param power The power to spin the intake at
     * @param durationMills The duration to spin the intake
     */
    public void outtakeForDuration(double power, long durationMills) {
        long start = System.currentTimeMillis();

        while (start + durationMills < System.currentTimeMillis()) {
            outtake(power);
        }
    }

    /**
     * Spins the intake in the outtake direction (REVERSE) for the specified amount of time
     * at the power defined by DEFAULT_OUTTAKE_POWER
     * @param durationMills The duration to spin the intake
     */
    public void outtakeForDuration(long durationMills) {
        outtakeForDuration(DEFAULT_OUTTAKE_POWER, durationMills);
    }

    /**
     * Stops the intake by setting the power to 0
     */
    public void stop() {
        intake.setPower(0);
    }

    /**
     * Displays debug information for the intake. To save loop time, this function DOES NOT call
     * telemetry.update().
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Intake Debug");

        telemetry.addData(
                "Intake direction",
                intake.getDirection());
        telemetry.addData(
                "Intake power",
                intake.getPower());
        telemetry.addData(
                "Intake current Amps",
                intake.getCurrent(AMPS));
    }
}
