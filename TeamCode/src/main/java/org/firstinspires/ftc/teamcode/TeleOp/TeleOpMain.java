package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.*;
import org.firstinspires.ftc.teamcode.TeleOp.Utility.ElevatorPositions;
import static org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp(name = "TeleOp 2023/2024 - Use This One", group = "Production")
public class TeleOpMain extends OpMode {

    private DriveBase driveBase;
    private Arm arm;
    private Intake intake;
    private Hanger hanger;
    private Launcher launcher;
    private boolean endGame = false;

    private EndGameState endGameState = EndGameState.IDLE;

    private enum EndGameState {
        IDLE,
        LAUNCH_POSITION,
        HANGING_POSITION,
        HUNG
    }


    private void normal() {
        driveBase.run(gamepad1);

        arm.checkLimitSwitch();

        if (gamepad2.left_bumper) {
            intake.intake();
        } else if (gamepad2.right_bumper) {
            intake.hold();
        } else if (gamepad2.right_trigger > 0.9) {
            intake.outtake();
        } else if (gamepad2.dpad_down) {
            arm.zero();
        } else if (gamepad2.dpad_up) {
            arm.moveToPosition(MED_EXTEND_POSITION, 0, WRIST_LEVEL_POSITION);
        } else if (gamepad2.cross) {
            arm.moveToPosition(BOTTOM_EXTEND_POSITION, BOTTOM_ROT_POSITION, BOTTOM_WRIST_POSITION);
        } else if (gamepad2.square) {
            arm.moveToPosition(LOW_EXTEND_POSITION, LOW_ROT_POSITION, LOW_WRIST_POSITION);
        } else if (gamepad2.circle) {
            arm.moveToPosition(MED_EXTEND_POSITION, MED_ROT_POSITION, MED_WRIST_POSITION);
        } else if (gamepad2.triangle) {
            arm.moveToPosition(HIGH_EXTEND_POSITION, HIGH_ROT_POSITION, HIGH_WRIST_POSITION);
        } else if (gamepad2.options) {
            arm.moveToPosition(TOP_EXTEND_POSITION, TOP_ROT_POSITION, TOP_WRIST_POSITION);
        }
    }



    private void endGame() {
        driveBase.run(gamepad1);

        arm.checkLimitSwitch();

        switch (endGameState) {
            case IDLE:
                if (gamepad2.left_bumper) {
                    endGameState = EndGameState.HANGING_POSITION;
                    break;
                } else if (gamepad2.right_bumper) {
                    endGameState = EndGameState.LAUNCH_POSITION;
                    break;
                }
            case HANGING_POSITION:
                arm.rotate(HANG_POSITION);

                if (gamepad2.right_bumper) {
                    endGameState = EndGameState.LAUNCH_POSITION;
                    break;
                } else if (gamepad2.dpad_down) {
                   arm.rotate(420);
                   endGameState = EndGameState.HUNG;
                   break;
                }
            case LAUNCH_POSITION:
                arm.rotate(LAUNCH_POSITION);

                if (gamepad2.left_bumper) {
                    endGameState = EndGameState.HANGING_POSITION;
                    break;
                } else if (gamepad2.left_trigger > 0.9 && gamepad2.left_bumper) {
                    launcher.release();
                    break;
                }
            case HUNG:
                driveBase.stop();
        }
    }
    
    @Override public void init() {
        driveBase = new DriveBase(hardwareMap);
        intake    = new Intake(hardwareMap);
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

        // Enter Endgame
        if (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8) { endGame = true; }

        // Leave Endgame
        if (endGame && gamepad2.left_trigger > 0.9 && gamepad2.right_trigger > 0.9) { endGame = false; }

        if (endGame) {
            telemetry.addLine("End Game");
            endGame();
        } else { normal(); }

        telemetry.update();
    }
}