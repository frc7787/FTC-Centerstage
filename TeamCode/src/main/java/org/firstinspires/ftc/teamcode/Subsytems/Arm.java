package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.DEFAULT_WORM_POWER;
import static org.firstinspires.ftc.teamcode.Properties.HANG_POS;
import static org.firstinspires.ftc.teamcode.Properties.HOMING_POWER;
import static org.firstinspires.ftc.teamcode.Properties.HUNG_POS;
import static org.firstinspires.ftc.teamcode.Properties.LAUNCH_POS;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to contain the arm subsystem. Arm contains two other subsystems elevator (extension) and
 * worm (rotation). This class has exclusive control over it's internal state, therefor neither it's
 * subsystem components, nor any outside class can influence it's state.
 * However, if the state of the arm needs to be read in order to control the state of another subsystem,
 * a getState method is available.
 */
public class Arm {
    final Elevator elevator;
    final Worm worm;

    HomingState homingState = HomingState.START;

    NormalArmState normalArmState = NormalArmState.HOME;
    EndGameState endGameArmState  = EndGameState.IDLE;
    MoveState moveState           = MoveState.IDLE;

    enum HomingState {
        START,
        HOMING_ELEVATOR,
        HOMING_WORM,
        COMPLETE
    }

    public enum NormalArmState {
        AT_POS,
        TO_POS,
        FROM_HOME,
        HOMING,
        HOME,
    }

    public enum EndGameState {
        IDLE,
        TO_IDLE,
        TO_HANGING_POS,
        HANGING_POS,
        TO_HUNG_POS,
        HUNG,
        TO_LAUNCHING_POS,
        LAUNCHING_POS,
    }

    enum MoveState {
        IDLE,
        ROTATING,
        EXTENDING,
        COMPLETE
    }

    /**
     * Arm subsystem constructor.
     * @param hardwareMap The hardwareMap you are using, likely "hardwareMap"
     */
    public Arm(@NonNull HardwareMap hardwareMap) {
        elevator = new Elevator(hardwareMap);
        worm     = new Worm(hardwareMap);
    }

    /**
     * Initializes the arm by calling the zero functions on elevator and worm
     */
    public void init() {
        elevator.init();
        worm.init();
    }

    /**
     * Sets the elevator state to homing
     */
    public void setHoming() {
        normalArmState = NormalArmState.HOMING;
    }

    /**
     * Function to update the state of the arm during endgame
     */
    public void updateEndgame() {
        elevator.update();
        worm.update();

        if (worm.is_busy()) {
            if (worm.targetPos() == 0) {
                endGameArmState = EndGameState.TO_IDLE;
            } else if (worm.targetPos() == LAUNCH_POS) {
                endGameArmState = EndGameState.TO_LAUNCHING_POS;
            } else if (worm.targetPos() == HANG_POS) {
                endGameArmState = EndGameState.TO_HANGING_POS;
            } else if (worm.targetPos() == HUNG_POS) {
                endGameArmState = EndGameState.TO_HUNG_POS;
            }
        } else {
            switch (endGameArmState) {
                case TO_IDLE:
                    endGameArmState = EndGameState.IDLE;
                    break;
                case TO_HANGING_POS:
                    endGameArmState = EndGameState.HANGING_POS;
                    break;
                case TO_LAUNCHING_POS:
                    endGameArmState = EndGameState.LAUNCHING_POS;
                    break;
                case TO_HUNG_POS:
                    endGameArmState = EndGameState.HUNG;
                    break;
            }

        }
    }

    /**
     * Function to update the state of the arm during the normal period of the game.
     */
    public void update() {
        elevator.update();
        worm.update();

        if (normalArmState == NormalArmState.HOMING) {
            home();
            return;
        }

        if (is_busy()) {
            if (normalArmState == NormalArmState.HOME || normalArmState == NormalArmState.FROM_HOME) {
                normalArmState = NormalArmState.FROM_HOME;
            } else {
                normalArmState = NormalArmState.TO_POS;
            }
        } else {
           if (normalArmState != NormalArmState.HOME) {
                normalArmState = NormalArmState.AT_POS;
           }
        }
    }

    /**
     * @return The current state of the arm.
     */
    public NormalArmState getNormalArmState() {
        return normalArmState;
    }

    public EndGameState endGameState() {
        return endGameArmState;
    }

    /**
     * @return worm_is_busy || elevator_is_busy
     */
    public boolean is_busy() { return worm.is_busy() || elevator.is_busy(); }

    public void moveToPosEndgame(int rotPos) {
        updateEndgame();

        worm.rotate(rotPos);
    }

    /**
     * Moves the arm to a specified position. Note this function updates the state of the elevator.
     *
     * @param extPos The position to extend the elevator to
     * @param rotPos The position to rotate the worm to
     */
    public void moveToPosition(int extPos, int rotPos) {
        update();

        if (normalArmState == NormalArmState.FROM_HOME) {
            switch (moveState) {
                case IDLE:
                    moveState = MoveState.ROTATING;
                    break;
                case ROTATING:
                    worm.power(DEFAULT_WORM_POWER);

                    if (worm.pos() >= 500) {
                        moveState = MoveState.EXTENDING;
                        break;
                    }
                case EXTENDING:
                    worm.rotate(rotPos);
                    elevator.extend(extPos);

                    if (worm.pos() == worm.targetPos() && elevator.pos() == elevator.targetPos()) {
                        moveState = MoveState.COMPLETE;
                        break;
                    }
                case COMPLETE:
                    moveState = MoveState.IDLE;
                    break;

            }
        } else {
            worm.rotate(rotPos);
            elevator.extend(extPos);
        }
    }

    /**
     * Runs the arm homing sequence
     */
    public void home() {
        // Cancel homing if we start moving anywhere else
        if (elevator.targetPos() != 0 || worm.targetPos() != 0) {
            homingState = HomingState.COMPLETE;
        }

        switch (homingState) {
            case START:
                homingState = HomingState.HOMING_ELEVATOR;
                break;
            case HOMING_ELEVATOR:
                if (elevator.limitSwitchIsPressed()) {
                    homingState = HomingState.HOMING_WORM;
                }
                elevator.power(HOMING_POWER);
                break;
            case HOMING_WORM:
                if (worm.limitSwitchIsPressed()) {
                    homingState = HomingState.COMPLETE;
                }
                worm.power(HOMING_POWER);
                break;
            case COMPLETE:
                break;
        }
    }

    /**
     * Displays debug info for the arm subsystem
     * @param telemetry The telemetry you are using to display the data
     */
    public void debug(@NonNull Telemetry telemetry) {
        elevator.debug(telemetry);
        worm.debug(telemetry);

        telemetry.addData("Arm Normal State", normalArmState);
        telemetry.addData("Arm EndGame State", endGameArmState);
        telemetry.addData("Current Arm Move Position", moveState);
        telemetry.addData("Current homing state", homingState);

        telemetry.addData("Current Arm Amps", getArmCurrentAmps());
    }

    public void disable() {
        worm.disable();
        elevator.disable();
    }

    /**
     * Gets the current draw of the elevator and worm motor in amps
     */
    public double getArmCurrentAmps() { return getElevatorCurrentAmps() + getWormCurrentAmps(); }

    /**
     * Gets the current draw of the elevator motor in amps
     */
    public double getElevatorCurrentAmps() { return elevator.currentAmps(); }

    /**
     * Gets the current draw of the worm motor in amps
     */
    public double getWormCurrentAmps() { return worm.getCurrent(); }
}
