package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_INTAKE_BELT_POWER;
import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_OUTTAKE_BELT_POWER;
import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_OUTTAKE_POWER;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to contain intake subsystem. Not to be confused
 * with the DeliveryTray subsystem.
 */
public class Intake {
    final DcMotorImplEx intakeMotor;
    final CRServoImplEx beltServo;

    /**
     * Creates a new instance of the intake subsystem
     * @param hardwareMap The hardware map, likely "hardwareMap"
     */
    public Intake(@NonNull HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "IntakeMotor");
        beltServo   = hardwareMap.get(CRServoImplEx.class, "IntakeBeltServo");
    }

    /**
     * Initializes the motor subsystem.
     * This sets the zero power behavior of the intake motor to float
     */
    public void init() {
        intakeMotor.setZeroPowerBehavior(FLOAT);
    }

    /**
     * Spins the intake in the intake direction (FORWARD) at the supplied power
     *
     * @param intakePower The power to give the intake motor
     */
    public void intake(double intakePower, double beltPower) {
        intakeMotor.setDirection(FORWARD);
        intakeMotor.setPower(intakePower);

        beltServo.setDirection(REVERSE);
        beltServo.setPower(beltPower);
    }

    /**
     * Spins the intake in the intake direction (FORWARD) by the powers defined by DEFAULT_INTAKE_POWER
     * and DEFAULT_BELT_POWER
     */
    public void intake() {
        intake(DEFAULT_INTAKE_POWER, DEFAULT_INTAKE_BELT_POWER);
    }

    /**
     * Spins the intake in the intake direction (FORWARD) for the specified amount of time
     *
     * @param durationMills The time to spin the intake for in milliseconds
     * @param intakePower The power to spin the intake at
     * @param beltPower The power to spin the belt at
     */
    public void intakeForDuration(long durationMills, double intakePower, double beltPower) {
        long start = System.currentTimeMillis();

        while (start + durationMills < System.currentTimeMillis()) {
            intake(intakePower, beltPower);
        }
    }

    /**
     * Spins the intake in the intake direction (FORWARD) for the specified amount of time at the
     * power specified by DEFAULT_INTAKE_POWER and DEFAULT_INTAKE_BELT_POWER
     *
     * @param durationMills The time to spin the intake for in milliseconds
     */
    public void intakeForDuration(long durationMills) {
       intakeForDuration(durationMills, DEFAULT_INTAKE_POWER, DEFAULT_INTAKE_BELT_POWER);
    }

    /**
     * Spins the intake in the outtake direction (REVERSE) at the specified power
     *
     * @param intakePower The power to spin the intake at
     * @param beltPower The power to spin the belt at
     */
    public void outtake(double intakePower, double beltPower) {
        intakeMotor.setDirection(REVERSE);
        intakeMotor.setPower(intakePower);

        beltServo.setDirection(FORWARD);
        beltServo.setPower(beltPower);
    }

    /**
     * Spins the intake in the outtake direction (REVERSE) at the power specified by
     * DEFAULT_OUTTAKE_POWER and DEFAULT_OUTTAKE_BELT_POWER
     */
    public void outtake() {
        outtake(DEFAULT_OUTTAKE_POWER, DEFAULT_OUTTAKE_BELT_POWER);
    }

    /**
     * Spins the intake in the outtake direction (REVERSE) for the specified amount of time.
     *
     * @param durationMills The duration to spin the intake
     * @param outtakePower The power to spin the intake at
     * @param beltPower The power to spin the belt at
     */
    public void outtakeForDuration(long durationMills, double outtakePower, double beltPower) {
        long start = System.currentTimeMillis();

        while (start + durationMills < System.currentTimeMillis()) {
            outtake(outtakePower, beltPower);
        }

        beltServo.getPwmRange();
    }

    /**
     * Spins the intake in the outtake direction (REVERSE) for the specified amount of time
     * at the power defined by DEFAULT_OUTTAKE_POWER and DEFAULT_OUTTAKE_BELT_POWER
     *
     * @param durationMills The duration to spin the intake
     */
    public void outtakeForDuration(long durationMills) {
        outtakeForDuration(durationMills, DEFAULT_OUTTAKE_POWER, DEFAULT_OUTTAKE_BELT_POWER);
    }

    /**
     * Stops the intake by setting the power to 0
     */
    public void stop() {
        intakeMotor.setPower(0);
        beltServo.setPower(0);
    }

    /**
     * Displays debug information for the intake. To save loop time, this function DOES NOT call
     * telemetry.update().
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Intake Debug");

        telemetry.addData(
                "Intake direction",
                intakeMotor.getDirection());
        telemetry.addData(
                "Intake power",
                intakeMotor.getPower());
        telemetry.addData(
                "Intake current Amps",
                intakeMotor.getCurrent(AMPS));

        telemetry.addData(
                "Belt Direction",
                beltServo.getDirection());
        telemetry.addData(
                "Belt Power",
                beltServo.getPower());
    }
}
