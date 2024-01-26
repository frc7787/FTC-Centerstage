package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.*;

import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import static org.firstinspires.ftc.teamcode.Properties.*;

import androidx.annotation.NonNull;

@TeleOp(name = "TeleOp 2023/2024 - Robot Centric", group = "Production")
@Disabled
public class TeleOpMain extends OpMode {
    DriveBase driveBase;
    Arm arm;
    Hanger hanger;
    Launcher launcher;
    Intake intake;
    DeliveryTray deliveryTray;

    GamePeriod gamePeriod     = GamePeriod.NORMAL;

    boolean hangerHookReleased = false;
    boolean intakeToggle       = false;
    boolean doorToggle         = false;


    enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    Gamepad prevGamepad2, currentGamepad2;

    /**
     * Set the LED color of gamepad1 and gamepad2. The color will continue until another color is set
     * @param r Red value
     * @param g Green value
     * @param b Blue value
     */
    void setGamepadLedColors(int r, int g, int b) {
        gamepad1.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
    }

    /**
     * Listens for gamepad input to move the tray
     * 
     * @param currentGamePad: The current state of the gamepad
     * @param prevGamepad: The state of the gamepad during the previous loop iteration
     */
    void listenForDeliveryTrayCommand(@NonNull Gamepad currentGamePad, @NonNull Gamepad prevGamepad) {
        if (currentGamePad.right_bumper && !prevGamepad.right_bumper) {
            doorToggle = !doorToggle;
        }

        if (doorToggle) {
            deliveryTray.openDoorToReleasePosition();
        } else {
            deliveryTray.closeDoor();
        }
    }

    /**
     * Listens for gamepad input to toggle the intake
     * 
     * @param currentGamepad: The current state of the gamepad
     * @param prevGamepad: The state of the gamepad during the previous loop iteration
     */
    void listenForIntakeCommand(@NonNull Gamepad currentGamepad, @NonNull Gamepad prevGamepad) {
        if (currentGamepad.left_bumper && !prevGamepad.left_bumper) {
            intakeToggle = !intakeToggle;
        }
        
        if (intakeToggle) {
            intake.intake();
        } else {
            intake.stop();
        }

    }

    /**
     * Listens for arm commands
     * @param currentGamePad The current copy of the gamepad state
     * @param prevGamePad The copy of the gamepad state from the previous loop iteration
     */
    void listenForNormalArmCommand(@NonNull Gamepad currentGamePad, @NonNull Gamepad prevGamePad) {
        if (currentGamePad.dpad_down) {
            arm.setHoming();
        } else if (currentGamePad.cross) {
            arm.moveToPosition(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (currentGamePad.square) {
            arm.moveToPosition(LOW_EXT_POS, LOW_ROT_POS);
        } else if (currentGamePad.circle) {
            arm.moveToPosition(MED_EXT_POS, MED_ROT_POS);
        } else if (currentGamePad.triangle) {
            arm.moveToPosition(HIGH_EXT_POS, HIGH_ROT_POS);
        } else if (currentGamePad.options) {
            arm.moveToPosition(TOP_EXT_POS, TOP_ROT_POS);
        }
    }

    void listenForEndgameArmCommand(@NonNull Gamepad currentGamepad, @NonNull Gamepad prevGamepad) {
        if (currentGamepad.left_bumper && !prevGamepad.left_bumper) {
            arm.moveToPosEndgame(LAUNCH_POS);
        } else if (currentGamepad.right_bumper && !prevGamepad.right_bumper) {
            arm.moveToPosEndgame(HANG_POS);
        } else if (currentGamepad.dpad_down && !prevGamepad.dpad_down) {
            arm.moveToPosEndgame(0);
        } else if (currentGamepad.dpad_up) {
            arm.moveToPosEndgame(HUNG_POS);
        }
    }

    /**
     * Code to run every iteration of the loop during the normal period (before endgame) of the match
     * @param currentGamepad The current state of the gamepad
     * @param prevGamepad The state of the gamepad during the previous loop iteration
     */
    void normalPeriodLoop(@NonNull Gamepad currentGamepad, @NonNull Gamepad prevGamepad) {
        arm.update();

        listenForIntakeCommand(currentGamepad, prevGamepad);
        listenForDeliveryTrayCommand(currentGamepad, prevGamepad);
        listenForNormalArmCommand(currentGamepad, prevGamepad);
    }

    /**
     * Code to run every iteration of the loop during the endgame period of the match
     * @param currentGamepad The current state of the gamepad
     * @param prevGamepad The state of the gamepad during the previous loop iteration
     */
    void endGameLoop(@NonNull Gamepad currentGamepad, @NonNull Gamepad prevGamepad) {
        arm.updateEndgame();

        switch (arm.endGameState()) {
            case HUNG:
                arm.disable();
                break;
            case HANGING_POS:
                listenForEndgameArmCommand(currentGamepad, prevGamepad);
                if (currentGamepad.cross && !prevGamepad.cross) {
                    hanger.release();
                    hangerHookReleased = true;
                }
                break;
            case LAUNCHING_POS:
                listenForEndgameArmCommand(currentGamepad, prevGamepad);
                if (currentGamepad.cross && !prevGamepad.cross) {
                    launcher.release();
                }
                break;
            default:
                listenForEndgameArmCommand(currentGamepad, prevGamepad);
                break;
        }
    }
    
    @Override public void init() {
        //RobotPropertyParser.loadProperties();

        prevGamepad2    = new Gamepad();
        currentGamepad2 = new Gamepad();

        driveBase    = new DriveBase(hardwareMap);
        hanger       = new Hanger(hardwareMap);
        launcher     = new Launcher(hardwareMap);
        arm          = new Arm(hardwareMap);
        intake       = new Intake(hardwareMap);
        deliveryTray = new DeliveryTray(hardwareMap);

        driveBase.init();
        arm.init();
        launcher.init();
        hanger.init();
        intake.init();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(AUTO);
        }
    }
    @Override public void loop() {
        arm.debug(telemetry);

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

               normalPeriodLoop(currentGamepad2, prevGamepad2);

               if (currentGamepad2.left_trigger > 0.5 && currentGamepad2.right_trigger > 0.5) {
                   gamePeriod = GamePeriod.ENDGAME;
               }
               break;
           case ENDGAME:
               telemetry.addLine("Endgame Period");
               endGameLoop(currentGamepad2, prevGamepad2);

               if (currentGamepad2.left_trigger > 0.9 && currentGamepad2.right_trigger > 0.9) {
                   gamePeriod = GamePeriod.NORMAL;
               }
               break;
       }

        telemetry.update();
    }


    @Override public void stop() {
        arm.disable();
    }
}