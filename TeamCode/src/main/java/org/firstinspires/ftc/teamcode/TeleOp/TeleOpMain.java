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

    // The following values should represent the robots starting configuration
    private EndGameState endGameState = EndGameState.IDLE;
    private GamePeriod gamePeriod     = GamePeriod.NORMAL; // This will likely never need to change
    private ArmState armState         = ArmState.RETRACTED; // If we end auto in a certain way, we should change this

    enum EndGameState {
        IDLE,
        HANGING,
        LAUNCHING,
        HUNG
    }

    enum ArmState {
        RETRACTED,
        EXTENDED,
        BOTTOM,
        LOW,
        MED,
        HIGH,
        TOP
    }

    enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    /**
     * Runs all of the intake code
     */
    void run_intake() {
        if (gamepad2.left_bumper) {
            intake.intake();
        } else if (gamepad2.right_bumper) {
            intake.hold();
        } else if (gamepad2.right_trigger > 0.9) {
            intake.outtake();
        }
    }


    void normal() {
        driveBase.run(gamepad1);

        arm.checkLimitSwitch();

        switch (armState) {
            case RETRACTED:
                run_intake();

                if (gamepad2.dpad_up) {
                    arm.extend(MED_EXTEND_POSITION);
                    armState = ArmState.EXTENDED;
                } else if (gamepad2.cross) {
                    arm.moveToBottomPosition(false);
                    armState = ArmState.BOTTOM;
                } else if (gamepad2.square) {
                    arm.moveToLowPosition(false);
                    armState = ArmState.LOW;
                } else if (gamepad2.circle) {
                    arm.moveToMedPosition(false);
                    armState = ArmState.MED;
                } else if (gamepad2.triangle) {
                    arm.moveToHighPosition(false);
                    armState = ArmState.HIGH;
                } else if (gamepad2.options) {
                    arm.moveToTopPosition(false);
                    armState = ArmState.TOP;
                }
                break;
            case EXTENDED:
                run_intake();

                if (gamepad2.dpad_down) {
                    intake.hold();
                    arm.zero();
                    armState = ArmState.RETRACTED;
                }
                if (gamepad2.cross) {
                    arm.moveToBottomPosition(false);
                    armState = ArmState.BOTTOM;
                } else if (gamepad2.square) {
                    arm.moveToLowPosition(false);
                    armState = ArmState.LOW;
                } else if (gamepad2.circle) {
                    arm.moveToMedPosition(false);
                    armState = ArmState.MED;
                } else if (gamepad2.triangle) {
                    arm.moveToHighPosition(false);
                    armState = ArmState.HIGH;
                } else if (gamepad2.options) {
                    arm.moveToTopPosition(false);
                    armState = ArmState.TOP;
                }
                break;
            case BOTTOM:
                run_intake();

                if (gamepad2.dpad_down) {
                    intake.hold();
                    arm.zero();
                    armState = ArmState.RETRACTED;
                } else if (gamepad2.dpad_up) {
                    arm.zero();
                    armState = ArmState.EXTENDED;
                } else if (gamepad2.square) {
                    arm.moveToLowPosition(false);
                    armState = ArmState.LOW;
                } else if (gamepad2.circle) {
                    arm.moveToMedPosition(false);
                    armState = ArmState.MED;
                } else if (gamepad2.triangle) {
                    arm.moveToHighPosition(false);
                    armState = ArmState.HIGH;
                } else if (gamepad2.options) {
                    arm.moveToTopPosition(false);
                    armState = ArmState.TOP;
                }
                break;
            case LOW:
                run_intake();

                if (gamepad2.dpad_down) {
                    arm.zero();
                    armState = ArmState.RETRACTED;
                } else if (gamepad2.dpad_up) {
                    arm.moveToExtendedPosition(true);
                    armState = ArmState.EXTENDED;
                } else if (gamepad2.cross) {
                    arm.moveToBottomPosition(true);
                    armState = ArmState.BOTTOM;
                } else if (gamepad2.circle) {
                    arm.moveToMedPosition(false);
                    armState = ArmState.MED;
                } else if (gamepad2.triangle) {
                    arm.moveToHighPosition(false);
                    armState = ArmState.HIGH;
                } else if (gamepad2.options) {
                    arm.moveToTopPosition(false);
                    armState = ArmState.TOP;
                }
                break;
            case MED:
                run_intake();

                if (gamepad2.dpad_down) {
                    arm.zero();
                    armState = ArmState.RETRACTED;
                } else if (gamepad2.dpad_up) {
                    arm.moveToExtendedPosition(true);
                    armState = ArmState.EXTENDED;
                } else if (gamepad2.cross) {
                    arm.moveToBottomPosition(true);
                    armState = ArmState.BOTTOM;
                } else if (gamepad2.triangle) {
                    arm.moveToHighPosition(false);
                    armState = ArmState.HIGH;
                } else if (gamepad2.options) {
                    arm.moveToTopPosition(false);
                    armState = ArmState.HIGH;
                }
                break;
            case HIGH:
                run_intake();

                if (gamepad2.dpad_down) {
                    intake.hold();
                    arm.zero();
                    armState = ArmState.RETRACTED;
                } else if (gamepad2.dpad_up) {
                    arm.moveToExtendedPosition(true);
                    armState = ArmState.EXTENDED;
                } else if (gamepad2.cross) {
                    arm.moveToBottomPosition(true);
                    armState = ArmState.BOTTOM;
                } else if (gamepad2.square) {
                    arm.moveToLowPosition(true);
                    armState = ArmState.LOW;
                } else if (gamepad2.circle) {
                    arm.moveToMedPosition(true);
                    armState = ArmState.MED;
                } else if (gamepad2.options) {
                    arm.moveToTopPosition(false);
                    armState = ArmState.TOP;
                }
                break;
            case TOP:
                run_intake();

                if (gamepad2.dpad_down) {
                    intake.hold();
                    arm.zero();
                    armState = ArmState.RETRACTED;
                } else if (gamepad2.dpad_up) {
                    arm.moveToExtendedPosition(true);
                    armState = ArmState.EXTENDED;
                } else if (gamepad2.cross) {
                    arm.moveToBottomPosition(true);
                    armState = ArmState.BOTTOM;
                } else if (gamepad2.square) {
                    arm.moveToLowPosition(true);
                    armState = ArmState.LOW;
                } else if (gamepad2.circle) {
                    arm.moveToMedPosition(true);
                    armState = ArmState.MED;
                } else if (gamepad2.triangle) {
                    arm.moveToHighPosition(true);
                    armState = ArmState.HIGH;
                }
                break;
        }
    }



    void endGame() {
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
                    arm.rotate(HUNG_POSITION);
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
                break;
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
                telemetry.addLine("Normal Period");
                normal();
                if (gamepad2.left_trigger > ENGAME_TRIGGER_SENSITIVITY && gamepad2.right_trigger > ENGAME_TRIGGER_SENSITIVITY) {
                    gamePeriod = GamePeriod.ENDGAME;
                }
                break;
            case ENDGAME:
                telemetry.addLine("End Game");
                endGame();
                if (gamepad2.left_trigger > ENGAME_TRIGGER_SENSITIVITY && gamepad2.right_trigger > ENGAME_TRIGGER_SENSITIVITY) {
                    gamePeriod = GamePeriod.NORMAL;
                }
                break;
        }
        telemetry.update();
    }

}