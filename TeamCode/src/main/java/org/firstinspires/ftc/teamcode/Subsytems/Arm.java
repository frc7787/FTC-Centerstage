package org.firstinspires.ftc.teamcode.Subsytems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Arm {

    public final Elevator elevator;
    public final Worm worm;
    public final Wrist wrist;

    public Arm(@NonNull HardwareMap hardwareMap) {
        elevator = new Elevator(hardwareMap);
        worm     = new Worm(hardwareMap);
        wrist    = new Wrist(hardwareMap);
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
