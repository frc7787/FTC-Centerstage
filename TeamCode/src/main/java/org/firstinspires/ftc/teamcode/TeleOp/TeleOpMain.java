package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

    private boolean isIntaking = false;

    private enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    private Gamepad prevGamepad2, currentGamepad2;

    private LED LEDOne, LEDTwo;

    private void EndgameLoop() {
       if (gamepad2.left_bumper) {
           Arm.setTargetPos(0, LAUNCH_POS);
       } else if (gamepad2.right_bumper) {
           Arm.setTargetPos(0, HANG_POS);
       }else if (gamepad2.dpad_down&& Arm.getWormTargetPos() == HANG_POS) {
           Arm.setTargetPos(0, -300);
       }

       if (gamepad2.left_trigger > 0.9 && Arm.getWormTargetPos() == LAUNCH_POS) {
           Auxiliaries.releaseLauncher();
       }

       if (gamepad2.right_trigger > 0.9 && Arm.getWormTargetPos() == HANG_POS) {
           Auxiliaries.releaseHanger();
       }
    }

    private void normalPeriodLoop() {
        if (gamepad1.dpad_up) {
            Auxiliaries.movePixelPlacerToMosiacFixingPositionLeft();
        } else if (gamepad1.dpad_down) {
            Auxiliaries.retractPixelPlacerLeft();
        }

        if (gamepad1.triangle) {
            Auxiliaries.movePixelPlacerToMosiacFixingPositionRight();
        } else if (gamepad1.cross) {
            Auxiliaries.retractPixelPlacerRight();
        }

        // Intake and delivery tray logic
        if (Arm.getWormPos()<10){
            if (gamepad2.left_trigger > 0.9) {
                isIntaking = true;
                Intake.intake();
                Arm.setDoorPos(TRAY_DOOR_OPEN_POS);
            } else if (gamepad2.left_trigger < 0.9 && gamepad2.left_trigger > 0.5) {
                isIntaking = true;
                Intake.intake();
                Arm.setDoorPos(TRAY_DOOR_CLOSED_POS);
            } else if (gamepad2.right_trigger > 0.9) {
                isIntaking = false;
                Intake.outtake();
            } else {
                isIntaking = false;
                Intake.stop();
                Arm.setDoorPos(TRAY_DOOR_CLOSED_POS);
            }
        } else {
            isIntaking = false;
            Intake.stop();

            if (gamepad2.right_bumper) {
                Arm.setDoorPos(TRAY_DOOR_OPEN_POS);
            } else {
                Arm.setDoorPos(TRAY_DOOR_CLOSED_POS);
            }
        }

        // Arm Logic
        if (gamepad2.dpad_down) {
            Arm.setTargetPos(0,0);
        } else if (gamepad2.cross) {
            Arm.setTargetPos(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (gamepad2.square) {
            Arm.setTargetPos(LOW_EXT_POS, LOW_ROT_POS);
        } else if (gamepad2.circle) {
            Arm.setTargetPos(MED_EXT_POS, MED_ROT_POS);
        } else if (gamepad2.triangle) {
            Arm.setTargetPos(HIGH_EXT_POS, HIGH_ROT_POS);
        }
    }
    
    @Override public void init() {
        currentGamepad2 = new Gamepad();
        prevGamepad2    = new Gamepad();

        Intake.init(hardwareMap);
        Auxiliaries.init(hardwareMap);
        DriveBase.init(hardwareMap);
        Arm.init(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(AUTO);
        }

        LEDOne = hardwareMap.get(LED.class, "LEDOne");
        LEDTwo = hardwareMap.get(LED.class, "LEDTwo");

        LEDOne.enable(false);
        LEDTwo.enable(false);
    }

    @Override public void loop() {
        Arm.update(isIntaking);

        prevGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        double drive  = gamepad1.left_stick_y * -1.0; // Left stick y is inverted
        double strafe = gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

       DriveBase.driveManualRobotCentric(drive, strafe, turn);

       switch (gamePeriod) {
           case NORMAL:
               gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
               gamepad2.setLedColor(0,0,255, Gamepad.LED_DURATION_CONTINUOUS);

               LEDOne.enable(false);
               LEDTwo.enable(false);

               normalPeriodLoop();

               if (gamepad2.share) {
                   gamePeriod = GamePeriod.ENDGAME;
                   gamepad1.rumble(1, 1, 1000);
                   gamepad2.rumble(1, 1, 1000);
               }

               break;
           case ENDGAME:
               gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
               gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);

               LEDOne.enable(true);
               LEDTwo.enable(true);

               EndgameLoop();
               break;
       }

    }
}