package org.firstinspires.ftc.teamcode.Subsytems;

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
    private final Elevator elevator;
    private final Worm worm;

    private int safetyLimit  = 2150;// worm safety limit 550 is a guess
    private int rotTargetPos = 0;
    private int extTargetPos = 0;

    HomingState homingState = HomingState.START;
    NormalArmState normalArmState = NormalArmState.UNKNOWN;
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
        HOMING,
        UNKNOWN,
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
        homingState=HomingState.START;
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
        moveArmToTarget();

        if (normalArmState == NormalArmState.HOMING) {
            home();
        }
    }


    public void moveToPosEndgame(int rotPos) { //to fix
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
        extTargetPos=extPos;
        rotTargetPos=rotPos;
    }
    private void moveArmToTarget() {

        switch (normalArmState){
            case UNKNOWN:
                setHoming();
                break;
            case AT_POS:
                if (extTargetPos>0 && worm.currentPos()<safetyLimit){
                    worm.rotate(safetyLimit);
                    rotTargetPos=Math.max(safetyLimit,rotTargetPos);
                }
                else{
                    worm.rotate(rotTargetPos);
                    elevator.extend(extTargetPos);
                    if (elevator.is_busy()||worm.is_busy()){
                        normalArmState=NormalArmState.TO_POS;
                    }

                }
                break;
            case TO_POS:
                if (elevator.is_busy()||worm.is_busy()){
                    break;
                }
                else {
                    normalArmState=NormalArmState.AT_POS;
                }
                break;
            case HOMING:
                break;
        }

    }

    /**
     * Runs the arm homing sequence
     */
    private void home() {
        // Cancel homing if we start moving anywhere else
        if (elevator.targetPos() != 0 || worm.targetPos() != 0) {
            homingState = HomingState.COMPLETE;
        }

        switch (homingState) {
            case START:
                homingState = HomingState.HOMING_ELEVATOR;
                normalArmState=NormalArmState.HOMING;
                extTargetPos=0;
                rotTargetPos=0;
                break;
            case HOMING_ELEVATOR:// never do this until
                elevator.extend(-5000,HOMING_POWER);
                if (elevator.limitSwitchIsPressed()) {
                    homingState = HomingState.HOMING_WORM;
                    elevator.extend(0,0);
                }

                break;
            case HOMING_WORM:
                worm.rotate(-5000,HOMING_POWER);
                if (worm.limitSwitchIsPressed()) {
                    homingState = HomingState.COMPLETE;
                    worm.rotate(0,0);
                }

                break;
            case COMPLETE:
                normalArmState=NormalArmState.AT_POS;
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

// Status update methods

    public boolean elevatorLimitSwitchIsPressed() { return elevator.limitSwitchIsPressed(); }

    public boolean wormLimitSwitchIsPressed() { return worm.limitSwitchIsPressed(); }

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
    public double getWormCurrentAmps() { return worm.currentAmps(); }
    public int getWormPos() {
        return worm.currentPos();
    }

    /**
     * @return The current state of the arm.
     */
    public NormalArmState getNormalArmState() {
        return normalArmState;
    }

    public void powerWorm(double power) { worm.power(power); }

    public EndGameState endGameState() {
        return endGameArmState;
    }

    /**
     * @return worm_is_busy || elevator_is_busy
     */
    public boolean is_busy() { return worm.is_busy() || elevator.is_busy(); }

}
