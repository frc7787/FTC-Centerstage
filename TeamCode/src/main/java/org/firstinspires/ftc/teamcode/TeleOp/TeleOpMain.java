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

    private EndGameState endGameState = EndGameState.IDLE;
    private GamePeriod gamePeriod     = GamePeriod.NORMAL;

    enum EndGameState {
        IDLE,
        HANGING,
        LAUNCHING,
        HUNG
    }

    enum GamePeriod {
        NORMAL,
        ENDGAME
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

        arm.checkLimitSwitch(); // We want to stop the motors if the limit switch is pressed

        switch (endGameState) {
            case IDLE:
                if (gamepad2.left_bumper)  {
                    endGameState = EndGameState.HANGING;
                } else if (gamepad2.right_bumper) {
                    endGameState = EndGameState.LAUNCHING;
                }
                break;
            case HANGING:
                arm.rotate(HANG_POSITION);

                if (gamepad2.right_bumper) {
                    endGameState = EndGameState.LAUNCHING;
                } else if (gamepad2.dpad_down) {
                    arm.rotate(420);
                    endGameState = EndGameState.HUNG;
                } else if (gamepad2.cross) {
                    hanger.release();
                }
                break;
            case LAUNCHING:
                arm.rotate(LAUNCH_POSITION);

                if (gamepad2.left_bumper) {
                    endGameState = EndGameState.HANGING;
                } else if (gamepad2.cross) {
                    launcher.release();
                }
                break;
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
        switch (gamePeriod) {
            case NORMAL:
                normal();
                if (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8) {
                    gamePeriod = GamePeriod.ENDGAME;
                }
                break;
            case ENDGAME:
                telemetry.addLine("End Game");
                endGame();
                if (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8) {
                    gamePeriod = GamePeriod.NORMAL;
                }
        }
        telemetry.update();
    }
}