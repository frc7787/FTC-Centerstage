package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Properties.INTAKE_POWER;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private final DcMotorImplEx intake;

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
     * Spins the intake
     */
    public void intake() {
       intake.setPower(INTAKE_POWER);
    }

    public void intake(long duration) {
        long start = System.currentTimeMillis();

        while (start + duration < System.currentTimeMillis()) {
            intake();
        }
    }

    public void outtake(double power) {
        intake.setDirection(REVERSE);
        intake.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
    }


    /**
     * Displays debug information for the intake
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Intake Debug");

        telemetry.addData("Intake direction", intake.getDirection());
        telemetry.addData("Intake power", intake.getPower());
    }
}
