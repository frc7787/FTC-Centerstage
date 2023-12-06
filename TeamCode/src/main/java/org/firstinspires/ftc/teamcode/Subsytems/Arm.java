package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.HOMING_POWER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public final Elevator elevator;
    public final Worm worm;
    public final Intake intake;

    public boolean isHoming = false;

    private HomingState homingState = HomingState.START;

    public enum HomingState {
        START,
        HOMING_ELEVATOR,
        HOMING_WORM,
        COMPLETE
    }

    /**
     * Arm subsystem constructor.
     * @param hardwareMap The hardwareMap you are using, likely "hardwareMap"
     */
    public Arm(@NonNull HardwareMap hardwareMap) {
        elevator = new Elevator(hardwareMap);
        worm     = new Worm(hardwareMap);
        intake   = new Intake(hardwareMap);
    }

    /**
     * Initializes the arm by calling the init functions on elevator and worm
     */
    public void init() {
        elevator.init();
        worm.init();
    }

    /**
     * Function to update the arm every iteration of the loop. <br>
     * Checks to see if we should stop and reset the elevator or worm encoders.
     * Additionally, this function determines whether or not the arm should home.
     */
    public void update() {
        if (isHoming) { home(); }

        elevator.update();
        worm.update();
    }


    /**
     * @return worm_is_busy || elevator_is_busy
     */
    public boolean is_busy() { return worm_is_busy() || elevator_is_busy(); }

    /**
     * @return If the worm is busy
     */
    public boolean worm_is_busy() { return worm.is_busy(); }

    /**
     * @return If the elevator is busy
     */
    public boolean elevator_is_busy() { return elevator.is_busy(); }

    /**
     * Moves the elevator and worm to their zero positions
     */
    public void zero() {
        elevator.extend(0);
        worm.rotate(0);
    }

    /**
     * Moves the arm to a specified position
     * @param elevatorPosition The position to extend the elevator to
     * @param wormPosition The position to rotate the worm to
     */
    public void moveToPosition(int elevatorPosition, int wormPosition) {
        worm.rotate(wormPosition);
        elevator.extend(elevatorPosition);
    }

    /**
     * @return If the elevator limit switch is pressed
     */
    public boolean elevatorLimitSwitchIsPressed() { return elevator.limitSwitchIsPressed(); }

    /**
     * @return If the worm limit switch is pressed
     */
    public boolean wormLimitSwitchIsPressed() { return worm.limitSwitchIsPressed(); }

    /**
     * Rotates the worm to the input position
     * @param position The position to rotate the worm to
     */
    public void rotate(int position) { worm.rotate(position); }

    /**
     * Extends the elevator to the input position
     * @param position The position to extend the elevator to
     */
    public void extend(int position) { elevator.extend(position); }

    /**
     * Sets the intake to intake at the speed defined by the INTAKE_SPEED property
     */
    public void intake() { intake.intake(); }

    /**
     * Sets the intake to outtake at the speed defined by the OUTTAKE_SPEED property
     */
    public void outtake() { intake.outtake(); }

    /**
     * Gets the target position of the arm
     * @return The target position of the arm [WormPos, ElevatorPos]
     */
    public int[] getTargetPosition() {
        return new int[]{worm.getTargetPosition(), elevator.getTargetPosition()};
    }

    /**
     * @return The target position of the worm
     */
    public int getWormTargetPosition() { return worm.getTargetPosition(); }

    /**
     * @return The target position of the elevator
     */
    public int getElevatorTargetPosition() { return elevator.getTargetPosition(); }

    /**
     * Gets the current position of the arm
     * @return The current position of the arm [WormPos, ElevatorPos]
     */
    public int[] getCurrentPosition() {
        return new int[]{worm.getCurrentPosition(), elevator.getCurrentPosition()};
    }

    /**
     * @return The current position of the worm motor
     */
    public int getWormCurrentPosition() { return worm.getCurrentPosition(); }

    /**
     * @return The current position of the elevator motor.
     */
    public int getElevatorCurrentPosition() { return elevator.getCurrentPosition(); }

    /**
     * Sets the homing state to START
     */
    public void resetHomingState() { homingState = HomingState.START; }

    /**
     * @return The current state of the homing sequence
     */
    public HomingState getHomingState() { return homingState; }

    /**
     * Runs the arm homing sequence
     */
    public void home() {
        if (elevator.getTargetPosition() != 0 || worm.getTargetPosition() != 0) {
            homingState = HomingState.COMPLETE;
        }

        switch (homingState) {
            case START:
                homingState = HomingState.HOMING_ELEVATOR;
                break;
            case HOMING_ELEVATOR:
                if (elevatorLimitSwitchIsPressed()) {
                    homingState = HomingState.HOMING_WORM;
                }
                elevator.power(HOMING_POWER);
                break;
            case HOMING_WORM:
                if (wormLimitSwitchIsPressed()) {
                    homingState = HomingState.COMPLETE;
                }
                worm.power(HOMING_POWER);
                break;
            case COMPLETE:
                isHoming = false;
                break;
        }
    }
}
