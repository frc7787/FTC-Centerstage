package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.*;

import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import static org.firstinspires.ftc.teamcode.Properties.*;

@TeleOp(name = "Judging - No Drive base", group = "Production")
@Disabled
public class Judging extends OpMode {
    DriveBase driveBase;
    Hanger hanger;
    Launcher launcher;
    Intake intake;
    DeliveryTray deliveryTray;

    GamePeriod gamePeriod = GamePeriod.NORMAL;

    boolean intakeToggle = false;
    boolean doorToggle   = false;

    enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    Gamepad prevGamepad2, currentGamepad2;

    private void listenForDeliveryTrayCommand() {
        if (currentGamepad2.right_bumper && !prevGamepad2.right_bumper) {
            doorToggle = !doorToggle;
        }

        if (doorToggle) {
            deliveryTray.openDoorToReleasePosition();
        } else {
            deliveryTray.closeDoor();
        }
    }


    private void listenForIntakeCommand() {
        if (currentGamepad2.left_bumper && !prevGamepad2.left_bumper) {
            intakeToggle = !intakeToggle;
        }

        if (intakeToggle) {
            deliveryTray.openDoorToReleasePosition();
        } else {
            deliveryTray.closeDoor();
        }

    }

    private void listenForNormalPeriodArmCommand() {
        if (gamepad2.dpad_down) {
            Arm.setHoming();
        } else if (gamepad2.cross) {
            Arm.setTargetPos(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (gamepad2.square) {
            Arm.setTargetPos(LOW_EXT_POS, LOW_ROT_POS);
        } else if (gamepad2.circle) {
            Arm.setTargetPos(MED_EXT_POS, MED_ROT_POS);
        } else if (gamepad2.triangle) {
            Arm.setTargetPos(HIGH_EXT_POS, HIGH_ROT_POS);
        } else if (gamepad2.options) {
            Arm.setTargetPos(TOP_EXT_POS, TOP_ROT_POS);
        }
    }

    private void listenForEndgameCommand() {
        if (gamepad2.left_bumper) {
            Arm.setTargetPos(0, LAUNCH_POS);
        } else if (gamepad2.right_bumper) {
            Arm.setTargetPos(0, HANG_POS);
        }

        if (gamepad2.left_trigger > 0.9) {
            launcher.release();
        }

        if (gamepad2.right_trigger > 0.9) {
            hanger.release();
        }
    }


    private void normalPeriodLoop() {
        listenForIntakeCommand();
        listenForDeliveryTrayCommand();
        listenForNormalPeriodArmCommand();
    }

    @Override public void init() {

        driveBase    = new DriveBase(hardwareMap);
        hanger       = new Hanger(hardwareMap);
        launcher     = new Launcher(hardwareMap);
        intake       = new Intake(hardwareMap);
        deliveryTray = new DeliveryTray(hardwareMap);

        driveBase.init();
        launcher.init();
        hanger.init();
        intake.init();

        Arm.init(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(AUTO);
        }
    }

    @Override public void loop() {
        Arm.debug(telemetry, true, true);

        prevGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        double drive  = gamepad1.left_stick_y * -1.0;
        double strafe = gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

        driveBase.driveManualRobotCentric(
                drive,
                strafe,
                turn
        );

        switch (gamePeriod) {
            case NORMAL:
                telemetry.addLine("Normal Period");

                normalPeriodLoop();

                if (gamepad2.options && gamepad2.share) {
                    gamePeriod = GamePeriod.ENDGAME;
                }
                break;
            case ENDGAME:
                telemetry.addLine("Endgame Period");
                listenForEndgameCommand();

                if (gamepad2.options && gamepad2.share) {
                    gamePeriod = GamePeriod.NORMAL;
                }
                break;
        }

        telemetry.update();
    }
}