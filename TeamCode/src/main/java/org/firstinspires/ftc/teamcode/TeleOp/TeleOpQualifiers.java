package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Properties.HANG_POS;
import static org.firstinspires.ftc.teamcode.Properties.LAUNCH_POS;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Subsytems.DeliveryTray;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Hanger;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.teamcode.Subsytems.Launcher;

import java.util.List;

@TeleOp(name = "TeleOp - Qualifiers")
public class TeleOpQualifiers extends OpMode {

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

    double wormPower         = 1.0;
    double elevatorHoldPower = -0.1;

    @Override public void init() {
        currentGamepad = new Gamepad();
        prevGamepad    = new Gamepad();

        intake       = new Intake(hardwareMap);
        driveBase    = new DriveBase(hardwareMap);
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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }


    @Override public void loop() {
        telemetry.addData("Elevator Pos", elevatorMotor.getCurrentPosition());
        telemetry.addData("Worm Pos", wormMotor.getCurrentPosition());

        prevGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);

        driveBase.driveManualRobotCentric(
                gamepad1.left_stick_y * -1.0,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        switch (period) {
            case NORMAL:
                telemetry.addLine("Period: Normal");
                if (currentGamepad.share && !prevGamepad.share) {
                    period = Period.ENDGAME;
                }

                if (wormLimitSwitch.isPressed()) {
                    if (currentGamepad.left_bumper) {
                        elevatorMotor.setPower(elevatorHoldPower);
                        deliveryTray.openDoorToIntakePos();
                        intake.intake();
                        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    } else {
                        intake.stop();
                        deliveryTray.closeDoor();
                        //elevatorMotor.setPower(0);
                        wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                } else {
                    intake.stop();
                    wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                if (elevatorMotor.getCurrentPosition() < 1300) {
                    deliveryTray.move_tray(0.0);
                } else {
                    deliveryTray.move_tray(0.5);
                }

                if (gamepad2.right_bumper) {
                    deliveryTray.openDoorToReleasePosition();
                }

                if (gamepad2.dpad_up) {
                    elevatorMotor.setPower(elevatorHoldPower);
                    if (elevatorLimitSwitch.isPressed() && wormMotor.getCurrentPosition() < 1500) {
                        deliveryTray.closeDoor();
                        wormMotor.setPower(wormPower);
                    } else {
                        wormMotor.setPower(0);
                    }
                } else if (gamepad2.dpad_down && elevatorLimitSwitch.isPressed() && !wormLimitSwitch.isPressed()) {
                    wormMotor.setPower(-wormPower);
                    deliveryTray.closeDoor();

                } else if (gamepad2.dpad_down && !elevatorLimitSwitch.isPressed() && !wormLimitSwitch.isPressed()) {
                    elevatorMotor.setPower(elevatorHoldPower);
                    wormMotor.setPower(0);
                } else {
                    wormMotor.setPower(0);
                    if (wormMotor.getCurrentPosition() > 1000) {
                        elevatorMotor.setPower(gamepad2.right_stick_y * -1.0);

                    }
                }


                break;
            case ENDGAME:
                telemetry.addLine("Endgame");
                if (currentGamepad.share && !prevGamepad.share) {
                    period = Period.NORMAL;
                }

                if (gamepad2.left_bumper) {
                    wormMotor.setTargetPosition(LAUNCH_POS);
                    wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormMotor.setPower(1);
                } else if (gamepad2.right_bumper) {
                    wormMotor.setTargetPosition(HANG_POS);
                    wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormMotor.setPower(1);
                } else if (gamepad2.dpad_down||gamepad2.dpad_up) {
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
                break;
        }
        telemetry.update();
    }
}
