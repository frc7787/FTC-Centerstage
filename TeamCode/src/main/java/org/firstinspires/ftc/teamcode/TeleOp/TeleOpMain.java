package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.*;
import org.firstinspires.ftc.teamcode.TeleOp.Utility.ElevatorPositions;
import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Subsytems.Arm.HomingState.*;

@TeleOp(name = "TeleOp 2023/2024 - Use This One", group = "Production")
public class TeleOpMain extends OpMode {

    private DriveBase driveBase;
    private Arm arm;
    private Hanger hanger;
    private Launcher launcher;

    // The following values should represent the robots starting configuration
    private EndGameState endGameState = EndGameState.IDLE;
    private NormalState normalState   = NormalState.AT_POSITION;
    private GamePeriod gamePeriod     = GamePeriod.NORMAL; // This will likely never need to change


    private enum EndGameState {
        IDLE,
        TO_HANGING_POSITION,
        HANGING_POSITION,
        HANGING,
        HUNG,
        TO_LAUNCH_POSITION,
        LAUNCH_POSITION
    }

    private enum NormalState {
        AT_POSITION,
        HOMING,
        MOVING_TO_POSITION
    }

    private enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    /**
     * Code to run elevator. <br>
     * This function sets the state of an elevator, however the state machine is located in the normal() function
     */
    private void run_elevator() {
        if (gamepad2.dpad_down) {
            normalState = NormalState.HOMING;
        } else if (gamepad2.dpad_up) {
            arm.moveToPosition(MED_EXTEND_POSITION, 0);
            normalState = NormalState.MOVING_TO_POSITION;
        } else if (gamepad2.cross) {
            arm.moveToPosition(BOTTOM_EXTEND_POSITION, BOTTOM_ROT_POSITION);
            normalState = NormalState.MOVING_TO_POSITION;
        } else if (gamepad2.square) {
            arm.moveToPosition(LOW_EXTEND_POSITION, LOW_ROT_POSITION);
            normalState = NormalState.MOVING_TO_POSITION;
        } else if (gamepad2.circle) {
            arm.moveToPosition(MED_EXTEND_POSITION, MED_ROT_POSITION);
            normalState = NormalState.MOVING_TO_POSITION;
        } else if (gamepad2.triangle) {
            arm.moveToPosition(HIGH_EXTEND_POSITION, HIGH_ROT_POSITION);
        } else if (gamepad2.options) {
            arm.moveToPosition(TOP_EXTEND_POSITION, TOP_ROT_POSITION);
        }
    }

    private void run_intake() {
        if (gamepad2.left_bumper) {
            arm.intake();
        } else if (gamepad2.right_bumper) {
            arm.close();
        } else if (gamepad2.left_trigger > INTAKE_TRIGGER_SENSITIVITY) {
            arm.outtake();
        }
    }

    private void normal() {
        switch (normalState) {
            case AT_POSITION:
                run_intake();
                run_elevator();
                break;
            case MOVING_TO_POSITION:
                run_intake();
                run_elevator();

                if (arm.getCurrentPosition() == arm.getTargetPosition()) {
                    normalState = NormalState.AT_POSITION;
                }
                break;
            case HOMING:
                arm.home();

                if (arm.getHomingState() == COMPLETE) {
                    normalState = NormalState.AT_POSITION;
                    arm.resetHomingState();
                }
                break;
        }
    }

    void endGame() {
        arm.checkLimitSwitch(); // We want to stop the motors if the limit switch is pressed

        switch (endGameState) {
            case IDLE:
                if (gamepad2.left_bumper) {
                    arm.rotate(HANG_POSITION);
                    endGameState = EndGameState.TO_HANGING_POSITION;
                } else if (gamepad2.right_bumper) {
                    arm.rotate(LAUNCH_POSITION);
                    endGameState = EndGameState.TO_LAUNCH_POSITION;
                }
                break;
            case TO_HANGING_POSITION:
                if (arm.worm_is_busy()) { endGameState = EndGameState.HANGING_POSITION; }
                if (gamepad2.right_bumper) {
                    arm.rotate(LAUNCH_POSITION);
                    endGameState = EndGameState.TO_LAUNCH_POSITION;
                }
            case TO_LAUNCH_POSITION:
                if (arm.worm_is_busy()) { endGameState = EndGameState.LAUNCH_POSITION; }
                if (gamepad2.left_bumper) {
                    arm.rotate(HANG_POSITION);
                    endGameState = EndGameState.TO_HANGING_POSITION;
                }
            case LAUNCH_POSITION:
                if (gamepad2.left_trigger > PLANE_LAUNCH_TRIGGER_SENSITIVITY && gamepad2.right_trigger > PLANE_LAUNCH_TRIGGER_SENSITIVITY ) {
                    launcher.release();
                } else if (gamepad2.left_bumper) {
                    arm.rotate(HANG_POSITION);
                    endGameState = EndGameState.TO_HANGING_POSITION;
                }
                break;
            case HANGING_POSITION:
                if (gamepad2.cross) {
                    hanger.release();
                } else if (gamepad2.dpad_up) {
                    arm.rotate(HUNG_POSITION);
                    endGameState = EndGameState.HUNG;
                } else if (gamepad2.right_bumper) {
                    arm.rotate(LAUNCH_POSITION);
                    endGameState = EndGameState.TO_LAUNCH_POSITION;
                }
            case HUNG:
                driveBase.stop();
                break;
        }
    }
    
    @Override public void init() {
        driveBase = new DriveBase(hardwareMap);
        hanger    = new Hanger(hardwareMap);
        launcher  = new Launcher(hardwareMap);
        arm       = new Arm(hardwareMap);

        arm.init();
        launcher.init();
        driveBase.init();
        hanger.init();

        ElevatorPositions.updateElevatorConstants();
    }


    @Override public void loop() {
        driveBase.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        switch (gamePeriod) {
            case NORMAL:
                normal();
                if (gamepad2.left_trigger > ENDGAME_TRIGGER_SENSITIVITY && gamepad2.right_trigger > ENDGAME_TRIGGER_SENSITIVITY) {
                    gamePeriod = GamePeriod.ENDGAME;
                }
                break;
            case ENDGAME:
                telemetry.addLine("End Game");
                endGame();
                if (gamepad2.left_trigger > ENDGAME_TRIGGER_SENSITIVITY && gamepad2.right_trigger > ENDGAME_TRIGGER_SENSITIVITY) {
                    gamePeriod = GamePeriod.NORMAL;
                }
                break;
        }
        telemetry.update();
    }

}