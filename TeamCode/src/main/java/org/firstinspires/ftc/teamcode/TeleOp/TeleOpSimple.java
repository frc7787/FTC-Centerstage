package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Properties.HANG_POS;
import static org.firstinspires.ftc.teamcode.Properties.HUNG_POS;
import static org.firstinspires.ftc.teamcode.Properties.LAUNCH_POS;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.DeliveryTray;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Hanger;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.teamcode.Subsytems.Launcher;

@TeleOp(name = "TeleOp Simple", group = "Production")
public class TeleOpSimple extends OpMode {

    Period period = Period.NORMAL;

    public enum Period {
        NORMAL,
        ENDGAME
    }

    Arm arm;
    Intake intake;
    DriveBase driveBase;
    DeliveryTray deliveryTray;
    Launcher launcher;
    Hanger hanger;

    DcMotorImplEx elevatorMotor, wormMotor;

    TouchSensor elevatorLimitSwitch, wormLimitSwitch;

    Gamepad currentGamepad, prevGamepad;

    boolean trayDoorIsOpen = false;
    boolean intakeToggle   = false;

    double wormPower         = 1.0;
    double elevatorHoldPower = -0.1;

    @Override public void init() {
        currentGamepad = new Gamepad();
        prevGamepad    = new Gamepad();

        intake = new Intake(hardwareMap);
        driveBase = new DriveBase(hardwareMap);
        deliveryTray = new DeliveryTray(hardwareMap);
        arm          = new Arm(hardwareMap);
        launcher     = new Launcher(hardwareMap);
        hanger       = new Hanger(hardwareMap);



        wormMotor     = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class, "ExtensionMotor");
        elevatorLimitSwitch = hardwareMap.get(TouchSensor.class, "ExtensionLimitSwitch");
        wormLimitSwitch     = hardwareMap.get(TouchSensor.class, "WormLimitSwitch");

        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.init();
        driveBase.init();
        deliveryTray.init();
        launcher.init();
        hanger.init();

        deliveryTray.closeDoor();
    }


    @Override public void loop() {
        prevGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);

        driveBase.driveManualRobotCentric(
                gamepad1.left_stick_y * -1.0,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        switch (period) {
            case NORMAL:
                if (wormLimitSwitch.isPressed()) {
                    if (currentGamepad.left_bumper && !prevGamepad.left_bumper) {
                        intakeToggle = !intakeToggle;
                    }

                    if (intakeToggle) {
                        deliveryTray.openDoor();
                        intake.intake();
                    } else {
                        deliveryTray.closeDoor();
                        intake.stop();
                    }
                }

                if (elevatorMotor.getCurrentPosition() < 1000) {
                    deliveryTray.move_tray(0.0);
                } else {
                    deliveryTray.move_tray(0.1);
                }

                if (gamepad2.right_bumper) {
                    deliveryTray.openDoor();
                } else {
                    deliveryTray.closeDoor();
                }

                telemetry.addData("Worm Current Position", wormMotor.getCurrentPosition());
                telemetry.addData("Elevator Current Position", elevatorMotor.getCurrentPosition());

                if (gamepad2.dpad_up) {
                    telemetry.addLine("We made it DPAD UP!");
                    if (elevatorLimitSwitch.isPressed() && wormMotor.getCurrentPosition() < 1500) {
                        deliveryTray.closeDoor();
                        telemetry.addLine("We are trying to power the worm");
                        wormMotor.setPower(wormPower);
                        elevatorMotor.setPower(elevatorHoldPower);
                    } else {
                        telemetry.addLine("We are trying to power the elevator");
                        wormMotor.setPower(0);
                        elevatorMotor.setPower(elevatorHoldPower);
                    }
                } else if (gamepad2.dpad_down && elevatorLimitSwitch.isPressed() && !wormLimitSwitch.isPressed()) {
                    telemetry.addLine("We made it to Dpad Down; bring the arm down");
                    wormMotor.setPower(-wormPower);
                    deliveryTray.closeDoor();
                    trayDoorIsOpen = false;
                } else if (gamepad2.dpad_down && !elevatorLimitSwitch.isPressed() && !wormLimitSwitch.isPressed()) {
                    telemetry.addLine("We made it to Dpad Down; trying to zero elevator");
                    elevatorMotor.setPower(elevatorHoldPower);
                    wormMotor.setPower(0);
                } else {
                    telemetry.addLine("Nothing happening. Set worm power to ZERO");
                    wormMotor.setPower(0);
                }

                if (wormMotor.getCurrentPosition() > 1400 ) {
                    if (elevatorMotor.getCurrentPosition() > 2700) {
                        elevatorMotor.setPower(Math.min(0, (gamepad2.left_stick_y * -1)));
                    }
                    else {
                        elevatorMotor.setPower(gamepad2.left_stick_y * -1);
                    }

                }
                if (gamepad2.options && gamepad2.options) {
                    period = Period.ENDGAME;
                }

                break;
            case ENDGAME:
                telemetry.addLine("Endgame");

//    void listenForEndgameArmCommand(@NonNull Gamepad currentGamepad, @NonNull Gamepad prevGamepad) {
//        if (currentGamepad.left_bumper && !prevGamepad.left_bumper) {
//            arm.moveToPosEndgame(LAUNCH_POS);
//        } else if (currentGamepad.right_bumper && !prevGamepad.right_bumper) {
//            arm.moveToPosEndgame(HANG_POS);
//        } else if (currentGamepad.dpad_down && !prevGamepad.dpad_down) {
//            arm.moveToPosEndgame(0);
//        } else if (currentGamepad.dpad_up) {
//            arm.moveToPosEndgame(HUNG_POS);
//        }
//    }
                if (gamepad2.left_bumper) {
                    wormMotor.setTargetPosition(LAUNCH_POS);
                    wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormMotor.setPower(1);
                } else if (gamepad2.right_bumper) {
                    wormMotor.setTargetPosition(HANG_POS);
                    wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormMotor.setPower(1);
                }else if (gamepad2.dpad_down||gamepad2.dpad_up) {
                    wormMotor.setTargetPosition(0);
                    wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormMotor.setPower(1);
                }

                if (wormMotor.getTargetPosition()==LAUNCH_POS &&!wormMotor.isBusy() && gamepad2.cross) {
                    launcher.release();
                }

                if (wormMotor.getTargetPosition()==HANG_POS &&!wormMotor.isBusy() && gamepad2.cross) {
                    hanger.release();
                }
                if (gamepad2.options && gamepad2.options) {
                    period = Period.NORMAL;
                }
        }

        telemetry.update();
    }
}
