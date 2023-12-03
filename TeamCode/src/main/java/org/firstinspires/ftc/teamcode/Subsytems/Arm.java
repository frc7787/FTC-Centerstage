package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.HOMING_POWER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public final Elevator elevator; /**
     So, the big question here, is if the elevator and worm should be run like their own subsystems, or just as a group of hardware controls.
     At the moment, Elevator has nothing in it to make it a "subsystem" it acts more as a group of methods for the arm subsystem
     with straight pass throughs to all of the hardware and minimal logical processing.
     It can stay as it's own class, but it should get some logical functionality so that it acts a bit more like a true subsystem.
     It seems excessive, but if it can be in a separate homing state,
     it should have it's own state machine that tracks homing or other features added later,
     and the Limit switch status or motor status should not be accessible to higher level subsystems
     **unless it is used as a sensor to control another action in a different subsystem
     (eg if you read the motor position and make a logical decision on how much power to send the drive subsystem)
     */
    public final Worm worm;
    public final Intake intake;

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
     * Checks the elevator and worm limit switches
     */
    public void checkLimitSwitch() {
        elevator.checkLimitSwitch();
        worm.checkLimitSwitch();
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
        checkLimitSwitch();
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
        switch (homingState) {
            case START:
                homingState = HomingState.HOMING_ELEVATOR;
                //you need to set your motor Modes here to RunUsingEncoder and set powers to zero
                // OR, and probably better would be to give your elevator and worm subsystems their own homing sequence and start them separately
                break;
            case HOMING_ELEVATOR:
                //keep calling the elevator.home() method once created
                if (elevatorLimitSwitchIsPressed()) { //ideally use an elevator state machine to let you know when it has completed homing
                    homingState = HomingState.HOMING_WORM;
                }
                elevator.power(HOMING_POWER);//this will not do anything if the motors are still set to run to position.
                break;
            case HOMING_WORM:
                //keep calling the worm.home() method once created
                if (wormLimitSwitchIsPressed()) {//ideally use a worm state machine to let you know when it has completed homing
                    homingState = HomingState.COMPLETE;
                }
                worm.power(HOMING_POWER); //this will not do anything if the motors are still set to run to position.
                break;
            case COMPLETE:
                break;
        }
    }
}
