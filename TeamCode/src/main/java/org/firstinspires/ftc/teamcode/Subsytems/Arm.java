package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Arm {

    public final Elevator elevator;
    public final Worm worm;
    public final Wrist wrist;
    public final Intake intake;

    public Arm(@NonNull HardwareMap hardwareMap) {
        elevator = new Elevator(hardwareMap);
        worm     = new Worm(hardwareMap);
        wrist    = new Wrist(hardwareMap);
        intake   = new Intake(hardwareMap);
    }

    public void init() {
        elevator.init();
        worm.init();
        wrist.init();
    }

    public void checkLimitSwitch() {
        elevator.checkLimitSwitch();
        worm.checkLimitSwitch();
    }

    public boolean worm_is_busy() { return worm.is_busy(); }

    public boolean elevator_is_busy() { return elevator.is_busy(); }

    public void zero() {
        elevator.extend(0);
        worm.rotate(0);
        wrist.level();
    }

    public void moveToPosition(int elevatorPosition, int wormPosition, double wristPosition) {
        worm.rotate(wormPosition);
        elevator.extend(elevatorPosition);
        wrist.setPosition(wristPosition);
    }

    public void moveToExtendedPosition(boolean downwards) {
        double rotPower = DEFAULT_WORM_POWER;
        double extPower = DEFAULT_ELEVATOR_POWER;
        if (downwards) {
            rotPower = DOWNWARDS_WORM_POWER;
            extPower = DOWNWARDS_ELEVATOR_POWER;
        }
        worm.rotate(0, rotPower);
        elevator.extend(MED_EXTEND_POSITION, extPower);
        wrist.setPosition(WRIST_LEVEL_POSITION);
    }

    public void moveToBottomPosition(boolean downwards) {
        double rotPower = DEFAULT_WORM_POWER;
        double extPower = DEFAULT_ELEVATOR_POWER;
        if (downwards) {
            rotPower = DOWNWARDS_WORM_POWER;
            extPower = DOWNWARDS_ELEVATOR_POWER;
        }

        worm.rotate(BOTTOM_ROT_POSITION, rotPower);
        elevator.extend(BOTTOM_EXTEND_POSITION, extPower);
        wrist.setPosition(BOTTOM_WRIST_POSITION);
    }
    public void moveToLowPosition(boolean downwards) {
        double rotPower = DEFAULT_WORM_POWER;
        double extPower = DEFAULT_ELEVATOR_POWER;
        if (downwards) {
            rotPower = DOWNWARDS_WORM_POWER;
            extPower = DOWNWARDS_ELEVATOR_POWER;
        }
        worm.rotate(LOW_ROT_POSITION, rotPower);
        elevator.extend(LOW_ROT_POSITION, extPower);
        wrist.setPosition(LOW_WRIST_POSITION);
    }

    public void moveToMedPosition(boolean downwards) {
        double rotPower = DEFAULT_WORM_POWER;
        double extPower = DEFAULT_ELEVATOR_POWER;
        if (downwards) {
            rotPower = DOWNWARDS_WORM_POWER;
            extPower = DOWNWARDS_ELEVATOR_POWER;
        }
        worm.rotate(MED_ROT_POSITION, rotPower);
        elevator.extend(MED_EXTEND_POSITION,extPower);
        wrist.setPosition(MED_WRIST_POSITION);
    }

    public void moveToHighPosition(boolean downwards) {
        double rotPower = DEFAULT_WORM_POWER;
        double extPower = DEFAULT_ELEVATOR_POWER;
        if (downwards) {
            rotPower = DOWNWARDS_WORM_POWER;
            extPower = DOWNWARDS_ELEVATOR_POWER;
        }
        worm.rotate(MED_ROT_POSITION, rotPower);
        elevator.extend(MED_EXTEND_POSITION, extPower);
        wrist.setPosition(MED_WRIST_POSITION);
    }

    public void moveToTopPosition(boolean downwards) {
        double rotPower = DEFAULT_WORM_POWER;
        double extPower = DEFAULT_ELEVATOR_POWER;
        if (downwards) {
            rotPower = DOWNWARDS_WORM_POWER;
            extPower = DOWNWARDS_ELEVATOR_POWER;
        }
        worm.rotate(TOP_ROT_POSITION, rotPower);
        elevator.extend(TOP_EXTEND_POSITION, extPower);
        wrist.setPosition(TOP_WRIST_POSITION);
    }

    public boolean extensionLimitSwitchIsPressed() { return elevator.limitSwitchIsPressed(); }

    public boolean rotationLimitSwitchIsPressed() { return worm.limitSwitchIsPressed(); }

    public void rotate(int position) { worm.rotate(position); }

    public void powerWorm(double power) { worm.power(power); }

    public void powerElevator(double power) { elevator.power(power); }

    public void extend(int position) { elevator.extend(position); }

    public void stop() {
        elevator.stop();
        worm.stop();
    }
}
