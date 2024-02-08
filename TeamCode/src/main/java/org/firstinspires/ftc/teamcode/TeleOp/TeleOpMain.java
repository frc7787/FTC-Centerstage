package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.*;

import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import static org.firstinspires.ftc.teamcode.Properties.*;

@TeleOp(name = "TeleOp - Provincials - Use This One", group = "Production")
public class TeleOpMain extends OpMode {
    /* Dead Wheels
     *  Left Dead Wheel:   Control Hub Port 1
     *  Right Dead Wheel:  Expansion Hub Port 1
     *  Center Dead Wheel: Expansion Hub Port 3
     *
     * Drive
     *   FrontLeftDriveMotor:  Control Hub Port 0
     *   FrontRightDriveMotor: Expansion Hub Port 2
     *   BackLeftDriveMotor:   Control Hub Port 1
     *   BackRightDriveMotor:  Expansion Hub Port 1
     *
     * Arm:
     *  ExtensionMotor:       Control Hub Port 3
     *  WormMotor:            Expansion Hub Port 0
     *  ExtensionLimitSwitch: Control Hub Digital Port 0
     *  WormLimitSwitch:      Control Hub Digital Port 1
     *
     * Intake:
     *  IntakeMotor: Expansion Hub Port
     *
     * Launcher:
     *  LauncherServo: Control Hub Servo Port 1
     * Hook:
     *  HangerServo: Control Hub Servo Port 0
     *
     * DeliveryTray:
     *  WristServo:     Expansion Hub Port 0
     *  LeftDoorServo:  Expansion Hub Port 1
     *  RightDoorServo: Expansion Hub Port 2
     */

    private GamePeriod gamePeriod = GamePeriod.NORMAL;

    private boolean doorToggle   = false;

    private double intakeTriggerValue;

    private enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    private Gamepad prevGamepad2, currentGamepad2;

    private void listenForDeliveryTrayCommand() {
        if (currentGamepad2.right_bumper && !prevGamepad2.right_bumper) {
            doorToggle = !doorToggle;
        }

        if (doorToggle) {
            Arm.setDoorPos(TRAY_DOOR_OPEN_POS);
        } else {
            Arm.setDoorPos(TRAY_DOOR_CLOSED_POS);
        }
    }


    private void listenForIntakeCommand() {
      intakeTriggerValue = currentGamepad2.left_trigger;

      if (intakeTriggerValue > 0.9) {
          Intake.intake();
          Arm.setDoorPos(TRAY_DOOR_OPEN_POS);
      } else if (intakeTriggerValue < 0.9 && intakeTriggerValue > 0.5) {
          Intake.intake();
          Arm.setDoorPos(TRAY_DOOR_CLOSED_POS);
      } else {
          Intake.stop();
          Arm.setDoorPos(TRAY_DOOR_CLOSED_POS);
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
           Launcher.release();
       }

       if (gamepad2.right_trigger > 0.9) {
           Hanger.release();
       }
    }


    private void normalPeriodLoop() {
        listenForNormalPeriodArmCommand();
        listenForIntakeCommand();
        listenForDeliveryTrayCommand();
    }
    
    @Override public void init() {
        currentGamepad2 = new Gamepad();
        prevGamepad2    = new Gamepad();

        Intake.init(hardwareMap);
        Launcher.init(hardwareMap);
        Hanger.init(hardwareMap);
        DriveBase.init(hardwareMap);
        Arm.init(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(AUTO);
        }
    }

    @Override public void loop() {
        telemetry.addData("Intake trigger value", intakeTriggerValue);

        Arm.update();
        Arm.debug(telemetry);

        prevGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        double drive  = gamepad1.left_stick_y * -1.0; // Left stick y is inverted
        double strafe = gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

       DriveBase.driveManualRobotCentric(drive, strafe, turn);

       switch (gamePeriod) {
           case NORMAL:
               telemetry.addLine("Normal Period");

               normalPeriodLoop();

               if (gamepad2.options && gamepad2.share) { gamePeriod = GamePeriod.ENDGAME; }
               break;
           case ENDGAME:
               telemetry.addLine("Endgame Period");

               listenForEndgameCommand();

               if (gamepad2.options && gamepad2.share) { gamePeriod = GamePeriod.NORMAL; }
               break;
       }

        telemetry.update();
    }
}