package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotPropertyParser;
import org.firstinspires.ftc.teamcode.Subsytems.*;

import static org.firstinspires.ftc.teamcode.Properties.*;

@TeleOp(name = "TeleOp 2023/2024 - Use This One", group = "Production")
public class TeleOpMain extends OpMode {

    private DriveBase driveBase;
    private Arm arm;
    private Hanger hanger;
    private Launcher launcher;

    // The following values should represent the robots starting configuration
    private EndGameState endGameState = EndGameState.IDLE;
    private ArmState armState         = ArmState.AT_POSITION;
    private GamePeriod gamePeriod     = GamePeriod.NORMAL;

    private boolean hangerHookReleased = false;
    private enum EndGameState {
        IDLE,
        TO_HANGING_POSITION,
        HANGING_POSITION,
        HUNG,
        TO_LAUNCH_POSITION,
        LAUNCH_POSITION
    }

    private enum ArmState {
        AT_POSITION,
        MOVING_TO_POSITION,
    }

    private enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    /**
     * Listens for gamepad input and moves arm to desired position. <br>
     * The state of the code is handled in the normal() function.
     */
    private void listenForArmCommand() {
        if (gamepad2.dpad_down) {
            arm.isHoming = true;
        } else if (gamepad2.dpad_up) {
            arm.moveToPosition(MED_EXT_POS, 0);
        } else if (gamepad2.cross) {
            arm.moveToPosition(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (gamepad2.square) {
            arm.moveToPosition(LOW_EXT_POS, LOW_ROT_POS);
        } else if (gamepad2.circle) {
            arm.moveToPosition(MED_EXT_POS, MED_ROT_POS);
        } else if (gamepad2.triangle) {
            arm.moveToPosition(HIGH_EXT_POS, HIGH_ROT_POS);
        } else if (gamepad2.options) {
            arm.moveToPosition(TOP_EXT_POS, TOP_ROT_POS);
        }
    }



    /**
     * Code to run in the normal period of the match.
     * This is where the arm state is handled
     */
    private void runNormalPeriod() {
        arm.update();

        if (gamepad2.left_bumper) { arm.intake(); }

        // This state machine is currently useless, we don't do anything based on the states
        switch (armState) {
            case AT_POSITION:
                listenForArmCommand();
                if (arm.is_busy()) { armState = ArmState.MOVING_TO_POSITION; }
                break;
            case MOVING_TO_POSITION:
                listenForArmCommand();
                if (!arm.is_busy()) { armState = ArmState.AT_POSITION; }
                break;
        }
    }

    /**
     * Code to be run in the endgame period.
     */
    void runEndGame() {
        switch (endGameState) {
            case IDLE:
                if (gamepad2.left_bumper) {
                    arm.rotate(HANG_POS);
                    endGameState = EndGameState.TO_HANGING_POSITION;
                } else if (gamepad2.right_bumper) {
                    arm.rotate(LAUNCH_POS);
                    endGameState = EndGameState.TO_LAUNCH_POSITION;
                }
                break;
            case TO_HANGING_POSITION:
                if (!arm.worm_is_busy()) { endGameState = EndGameState.HANGING_POSITION; }
                if (gamepad2.right_bumper) {
                    arm.rotate(LAUNCH_POS);
                    endGameState = EndGameState.TO_LAUNCH_POSITION;
                }
                break;
            case TO_LAUNCH_POSITION:
                if (!arm.worm_is_busy()) { endGameState = EndGameState.LAUNCH_POSITION; }
                if (gamepad2.left_bumper) {
                    arm.rotate(HANG_POS);
                    endGameState = EndGameState.TO_HANGING_POSITION;
                }
                break;
            case LAUNCH_POSITION:
                if (gamepad2.left_trigger > PLANE_LAUNCH_TRIGGER_SENSITIVITY && gamepad2.right_trigger > PLANE_LAUNCH_TRIGGER_SENSITIVITY ) {
                    launcher.release();
                } else if (gamepad2.left_bumper) {
                    arm.rotate(HANG_POS);
                    endGameState = EndGameState.TO_HANGING_POSITION;
                }
                break;
            case HANGING_POSITION:
                if (gamepad2.right_bumper) {
                    arm.rotate(LAUNCH_POS);
                    endGameState = EndGameState.TO_LAUNCH_POSITION;
                } else if (gamepad2.cross) {
                    hanger.release();
                    hangerHookReleased = true;
                } else if (gamepad2.dpad_down && hangerHookReleased) {
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
        driveBase.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        switch (gamePeriod) {
            case NORMAL:
                telemetry.addLine("Normal Period");
                runNormalPeriod();
                if (gamepad2.left_trigger > ENDGAME_TRIGGER_SENSITIVITY && gamepad2.right_trigger > ENDGAME_TRIGGER_SENSITIVITY) {
                    gamePeriod = GamePeriod.ENDGAME;
                }
                break;
            case ENDGAME:
                telemetry.addLine("End Game");
                runEndGame();
                if (gamepad2.left_trigger > ENDGAME_TRIGGER_SENSITIVITY && gamepad2.right_trigger > ENDGAME_TRIGGER_SENSITIVITY) {
                    gamePeriod = GamePeriod.NORMAL;
                }
                break;
        }
        telemetry.update();
    }

}