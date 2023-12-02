package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.HOMING_POWER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public final Elevator elevator;
    public final Worm worm;
    public final Intake intake;

    private HomingState homingState = HomingState.START;

    public enum HomingState {
        START,
        HOMING_ELEVATOR,
        HOMING_WORM,
        COMPLETE
    }

    public Arm(@NonNull HardwareMap hardwareMap) {
        elevator = new Elevator(hardwareMap);
        worm     = new Worm(hardwareMap);
        intake   = new Intake(hardwareMap);
    }

    public void init() {
        elevator.init();
        worm.init();
    }

    public void checkLimitSwitch() {
        elevator.checkLimitSwitch();
        worm.checkLimitSwitch();
    }

    public boolean is_busy() { return worm_is_busy() || elevator_is_busy(); }

    public boolean worm_is_busy() { return worm.is_busy(); }

    public boolean elevator_is_busy() { return elevator.is_busy(); }

    public void zero() {
        elevator.extend(0);
        worm.rotate(0);
    }

    public void moveToPosition(int elevatorPosition, int wormPosition) {
        worm.rotate(wormPosition);
        elevator.extend(elevatorPosition);
    }

    public boolean extensionLimitSwitchIsPressed() { return elevator.limitSwitchIsPressed(); }

    public boolean rotationLimitSwitchIsPressed() { return worm.limitSwitchIsPressed(); }

    public void rotate(int position) { worm.rotate(position); }

    public void extend(int position) { elevator.extend(position); }

    public void intake() { intake.intake(); }

    public void outtake() { intake.outtake(); }

    public int[] getTargetPosition() {
        return new int[]{worm.getTargetPosition(), elevator.getTargetPosition()};
    }

    public int getWormTargetPosition() { return worm.getTargetPosition(); }

    public int getElevatorTargetPosition() { return elevator.getTargetPosition(); }

    public int[] getCurrentPosition() {
        return new int[]{worm.getCurrentPosition(), elevator.getCurrentPosition()};
    }

    public int getWormCurrentPosition() { return worm.getCurrentPosition(); }

    public int getElevatorCurrentPosition() { return elevator.getCurrentPosition(); }

    public void resetHomingState() { homingState = HomingState.START; }

    public HomingState getHomingState() { return homingState; }

    public void home() {
        switch (homingState) {
            case START:
                homingState = HomingState.HOMING_ELEVATOR;
                break;
            case HOMING_ELEVATOR:
                if (extensionLimitSwitchIsPressed()) {
                    homingState = HomingState.HOMING_WORM;
                }
                elevator.power(HOMING_POWER);
                break;
            case HOMING_WORM:
                if (rotationLimitSwitchIsPressed()) {
                    homingState = HomingState.COMPLETE;
                }
                worm.power(HOMING_POWER);
                break;
            case COMPLETE:
                break;
        }
    }
}
