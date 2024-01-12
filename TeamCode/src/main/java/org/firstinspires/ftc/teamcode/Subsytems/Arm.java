package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.HOMING_POWER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    public final Elevator elevator;
    public final Worm worm;
    public final Intake intake;
    public final DeliveryTray delivery_tray;

    public boolean isHoming = false;

    private HomingState homingState = HomingState.START;

    private enum HomingState {
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
        delivery_tray = new DeliveryTray(hardwareMap);
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
        delivery_tray.raise_tray();
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
     * Spins the Intake
     */
    public void intake() { intake.intake(); }

    /**
     * Runs the arm homing sequence
     */
    public void home() {
        // Cancel homing if we start moving anywhere else
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

    /**
     * Displays debug info for the arm subsystem
     * @param telemetry The telemetry you are using to display the data
     */
    public void debug(@NonNull Telemetry telemetry) {
        elevator.debug(telemetry);
        worm.debug(telemetry);
    }

    /**
     * Gets the current draw of the elevator and worm motor in amps
     */
    public double getArmCurrentAmps() { return getElevatorCurrentAmps() + getWormCurrentAmps(); }

    /**
     * Gets the current draw of the elevator motor in amps
     */
    public double getElevatorCurrentAmps() { return elevator.getCurrentAmps(); }

    /**
     * Gets the current draw of the worm motor in amps
     */
    public double getWormCurrentAmps() { return worm.getWormCurrent(); }
}
