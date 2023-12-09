package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotPropertyParser;
import org.firstinspires.ftc.teamcode.Subsytems.*;

import static org.firstinspires.ftc.teamcode.Properties.*;

import androidx.annotation.NonNull;

@TeleOp(name = "TeleOp 2023/2024 - Use This One", group = "Production")
public class TeleOpMain extends OpMode {

    DriveBase driveBase;
    Arm arm;
    Hanger hanger;
    Launcher launcher;

    // The following values should represent the robots starting configuration
    EndGameState endGameState = EndGameState.IDLE;
    ArmState armState         = ArmState.AT_POSITION;
    GamePeriod gamePeriod     = GamePeriod.NORMAL;

    boolean hangerHookReleased = false;
    enum EndGameState {
        IDLE,
        TO_HANGING_POSITION,
        HANGING_POSITION,
        HUNG,
        TO_LAUNCH_POSITION,
        LAUNCH_POSITION
    }

    enum ArmState {
        AT_POSITION,
        MOVING_TO_POSITION,
    }

    enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    Gamepad prevGamepad2, currentGamepad2;

    /**
     * Listens for arm commands
     * @param currentGamePad The current copy of the gamepad state
     * @param prevGamePad The copy of the gamepad state from the previous loop iteration
     */
    void listenForNormalArmCommand(@NonNull Gamepad currentGamePad, @NonNull Gamepad prevGamePad) {
        if (currentGamePad.dpad_down && !prevGamePad.dpad_down) {
            arm.isHoming = true;
        } else if (currentGamePad.dpad_up && !prevGamePad.dpad_up) {
            arm.moveToPosition(MED_EXT_POS, 0);
        } else if (currentGamePad.cross && !prevGamePad.cross) {
            arm.moveToPosition(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (currentGamePad.square && !prevGamePad.square) {
            arm.moveToPosition(LOW_EXT_POS, LOW_ROT_POS);
        } else if (currentGamePad.circle && !prevGamePad.circle) {
            arm.moveToPosition(MED_EXT_POS, MED_ROT_POS);
        } else if (currentGamePad.triangle && !prevGamePad.triangle) {
            arm.moveToPosition(HIGH_EXT_POS, HIGH_ROT_POS);
        } else if (currentGamePad.options && !prevGamePad.options) {
            arm.moveToPosition(TOP_EXT_POS, TOP_ROT_POS);
        }
    }
    /**
     * Code to run in the normal period of the match.
     * This is where the arm state is handled
     */
    void normalPeriodLoop(@NonNull Gamepad currentGamepad, @NonNull Gamepad prevGamepad) {
        arm.update();

        if (gamepad2.left_bumper) { arm.intake(); }

        // This state machine is currently useless, we don't do anything based on the states
        switch (armState) {
            case AT_POSITION:
                listenForNormalArmCommand(currentGamepad, prevGamepad);
                if (arm.is_busy()) { armState = ArmState.MOVING_TO_POSITION; }
                break;
            case MOVING_TO_POSITION:
                listenForNormalArmCommand(currentGamepad, prevGamepad);
                if (!arm.is_busy()) { armState = ArmState.AT_POSITION; }
                break;
        }
    }

    /**
     * Code to be run in the endgame period.
     */
    void endGameLoop(@NonNull Gamepad currentGamepad, @NonNull Gamepad prevGamepad) {
        switch (endGameState) {
            case IDLE:
                if (currentGamepad.left_bumper && !prevGamepad.left_bumper) {
                    arm.rotate(HANG_POS);
                    endGameState = EndGameState.TO_HANGING_POSITION;
                } else if (currentGamepad.right_bumper && !prevGamepad.right_bumper) {
                    arm.rotate(LAUNCH_POS);
                    endGameState = EndGameState.TO_LAUNCH_POSITION;
                }
                break;
            case TO_HANGING_POSITION:
                if (!arm.worm_is_busy()) { endGameState = EndGameState.HANGING_POSITION; }
                if (currentGamepad.right_bumper && !prevGamepad.right_bumper) {
                    arm.rotate(LAUNCH_POS);
                    endGameState = EndGameState.TO_LAUNCH_POSITION;
                }
                break;
            case TO_LAUNCH_POSITION:
                if (!arm.worm_is_busy()) { endGameState = EndGameState.LAUNCH_POSITION; }
                if (currentGamepad.left_bumper && !prevGamepad.right_bumper) {
                    arm.rotate(HANG_POS);
                    endGameState = EndGameState.TO_HANGING_POSITION;
                }
                break;
            case LAUNCH_POSITION:
                if (gamepad2.left_trigger > PLANE_LAUNCH_TRIGGER_SENSITIVITY && gamepad2.right_trigger > PLANE_LAUNCH_TRIGGER_SENSITIVITY ) {
                    launcher.release();
                } else if (currentGamepad.left_bumper && !prevGamepad.left_bumper) {
                    arm.rotate(HANG_POS);
                    endGameState = EndGameState.TO_HANGING_POSITION;
                }
                break;
            case HANGING_POSITION:
                if (currentGamepad.right_bumper && !prevGamepad.right_bumper) {
                    arm.rotate(LAUNCH_POS);
                    endGameState = EndGameState.TO_LAUNCH_POSITION;
                } else if (currentGamepad.cross && !prevGamepad.cross) {
                    hanger.release();
                    hangerHookReleased = true;
                } else if (currentGamepad.dpad_down && hangerHookReleased) {
                    arm.rotate(HUNG_POS);
                }
                break;
            case HUNG:
                driveBase.stop();
                break;
        }
    }
    
    @Override public void init() {
        RobotPropertyParser.loadProperties();

        prevGamepad2    = new Gamepad();
        currentGamepad2 = new Gamepad();

        driveBase = new DriveBase(hardwareMap);
        hanger    = new Hanger(hardwareMap);
        launcher  = new Launcher(hardwareMap);
        arm       = new Arm(hardwareMap);

        arm.init();
        launcher.init();
        driveBase.init();
        hanger.init();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { module.setBulkCachingMode(AUTO); }
    }


    @Override public void loop() {
        prevGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        driveBase.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        switch (gamePeriod) {
            case NORMAL:
                gamepad1.setLedColor(0, 255, 0, 200);
                gamepad2.setLedColor(0, 255, 0, 200);
                normalPeriodLoop(currentGamepad2, prevGamepad2);
                if (gamepad2.left_trigger > ENDGAME_TRIGGER_SENSITIVITY && gamepad2.right_trigger > ENDGAME_TRIGGER_SENSITIVITY) {
                    gamePeriod = GamePeriod.ENDGAME;
                }
                break;
            case ENDGAME:
                gamepad1.setLedColor(128, 0, 128, 200);
                gamepad2.setLedColor(128, 0, 128, 200);
                endGameLoop(currentGamepad2, prevGamepad2);
                if (gamepad2.left_trigger > ENDGAME_TRIGGER_SENSITIVITY && gamepad2.right_trigger > ENDGAME_TRIGGER_SENSITIVITY) {
                    gamePeriod = GamePeriod.NORMAL;
                }
                break;
        }
        telemetry.update();
    }

}