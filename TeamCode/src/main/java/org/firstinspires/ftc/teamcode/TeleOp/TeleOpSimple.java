package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.DeliveryTray;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;

@TeleOp(name = "TeleOp Simple", group = "Production")
public class TeleOpSimple extends OpMode {
    Intake intake;
    DriveBase driveBase;
    DeliveryTray deliveryTray;

    DcMotorImplEx elevatorMotor, wormMotor;

    TouchSensor elevatorLimitSwitch, wormLimitSwitch;

    boolean trayDoorIsOpen = false;

    double wormPower         = 1.0;
    double elevatorHoldPower = 0.1;

    @Override public void init() {
        intake = new Intake(hardwareMap);
        driveBase = new DriveBase(hardwareMap);
        deliveryTray = new DeliveryTray(hardwareMap);

        wormMotor     = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class, "ExtensionMotor");
        elevatorLimitSwitch = hardwareMap.get(TouchSensor.class, "ExtensionLimitSwitch");
        wormLimitSwitch     = hardwareMap.get(TouchSensor.class, "WormLimitSwitch");

        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.init();
        driveBase.init();
        deliveryTray.init();

        deliveryTray.closeDoor();
    }

    @Override public void loop() {
        telemetry.addData("Worm Current Position", wormMotor.getCurrentPosition());
        telemetry.addData("Elevator Current Position", elevatorMotor.getCurrentPosition());

        if (gamepad2.dpad_up) {
            telemetry.addLine("We made it DPAD UP!");
            if (elevatorLimitSwitch.isPressed() && wormMotor.getCurrentPosition() < 1500) {
                telemetry.addLine("We are trying to power the worm");
                wormMotor.setPower(wormPower);
                elevatorMotor.setPower(elevatorHoldPower);
            } else {
                telemetry.addLine("We are trying to power the elevator");
                wormMotor.setPower(0);
                elevatorMotor.setPower(elevatorHoldPower);
            }
        } else if (gamepad2.dpad_down && elevatorLimitSwitch.isPressed() && !wormLimitSwitch.isPressed()) {
                telemetry.addLine("We made it to Dpad Down");
                wormMotor.setPower(-wormPower);
                deliveryTray.closeDoor();
                trayDoorIsOpen = false;
        } else {
            wormMotor.setPower(0);
        }

        if (wormMotor.getCurrentPosition() > 1400) {
            elevatorMotor.setPower(gamepad2.left_stick_y * -1);
        }
        telemetry.update();
    }
}
