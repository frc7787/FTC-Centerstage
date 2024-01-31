package org.firstinspires.ftc.teamcode.Subsytems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.teamcode.Properties.*;
import static org.firstinspires.ftc.teamcode.Subsytems.Utility.HomingState.*;
import static org.firstinspires.ftc.teamcode.Subsytems.Utility.NormalPeriodArmState.*;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsytems.Utility.HomingState;
import org.firstinspires.ftc.teamcode.Subsytems.Utility.NormalPeriodArmState;

public class Arm {
    private static final int WORM_SAFETY_LIMIT = 2150;

    private static DcMotorImplEx wormMotor, elevatorMotor;
    private static RevTouchSensor wormLimitSwitch, elevatorLimitSwitch;

    private static int elevatorTargetPos, wormTargetPos;

    private static NormalPeriodArmState normalPeriodArmState;
    private static HomingState homingState;

    /**
     * Initializes all of the hardware for the Arm, and resets the arm state
     * @param hardwareMap The hardware map you are using to get the hardware likely "hardwareMap"
     */
    public static void init(@NonNull HardwareMap hardwareMap) {
        wormLimitSwitch     = hardwareMap.get(RevTouchSensor.class, "WormLimitSwitch");
        elevatorLimitSwitch = hardwareMap.get(RevTouchSensor.class, "ElevatorLimitSwitch");

        wormMotor     = hardwareMap.get(DcMotorImplEx.class, "WormMotor");
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class, "ElevatorMotor");

        wormMotor.setMotorEnable();
        wormMotor.setMode(STOP_AND_RESET_ENCODER);
        wormMotor.setMode(RUN_USING_ENCODER);

        elevatorMotor.setMotorEnable();
        elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(RUN_USING_ENCODER);
        elevatorMotor.setDirection(REVERSE);

        elevatorTargetPos = 0;
        wormTargetPos     = 0;

        normalPeriodArmState = UNKNOWN;
        homingState          = IDLE;
    }

    /**
     * Function to update the state of the arm every loop, moves the arm to the target pos
     * and checks if the arm should be homing.
     */
    public static void update() {
        moveArmToTargetPos();

        if (normalPeriodArmState == HOMING) {
            home();
        }
    }

    /**
     * Command the arm to start the homing sequence
     */
    public static void setHoming() {
        normalPeriodArmState = HOMING;
        homingState          = START;
    }

    /**
     * Sets the target position of the arm
     * @param wormTargetPos The target position of the worm
     * @param elevatorTargetPos The target position of the elevator
     */
    public static void setTargetPos(int elevatorTargetPos, int wormTargetPos) {
        Arm.elevatorTargetPos = elevatorTargetPos;
        Arm.wormTargetPos     = wormTargetPos;
    }

    private static void moveArmToTargetPos() {
        switch (normalPeriodArmState) {
            case AT_POS:
                if (elevatorTargetPos > 0 && wormMotor.getCurrentPosition() < WORM_SAFETY_LIMIT) {
                    // If the target worm pos is greater than the safety limit we go there, if not we go to the safety limit.
                    rotateWorm(Math.max(wormTargetPos, WORM_SAFETY_LIMIT));
                } else {
                    rotateWorm(wormTargetPos);
                    extendElevator(elevatorTargetPos);

                    if (wormMotor.isBusy() || elevatorMotor.isBusy()) {
                        normalPeriodArmState = TO_POS;
                    }
                }
                break;
            case TO_POS:
                if (!elevatorMotor.isBusy() && !wormMotor.isBusy()) {
                    normalPeriodArmState = AT_POS;
                }
                break;
            case UNKNOWN:
                home();

        }
    }

    /**
     * Homes the arm and worm. First, it homes the elevator, then the worm after it is finished
     */
    public static void home() {
        if (elevatorMotor.getCurrentPosition() != 0 || wormMotor.getTargetPosition() != 0) {
            normalPeriodArmState = TO_POS;
            homingState = COMPLETE;
        }

        switch (homingState) {
            case START:
                homingState          = HOMING_ELEVATOR;
                normalPeriodArmState = HOMING;

                elevatorTargetPos = 0;
                wormTargetPos     = 0;
                break;
            case HOMING_ELEVATOR:
                extendElevator(-100);
                if (elevatorLimitSwitch.isPressed()) {
                    homingState = HOMING_WORM;
                    elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
                    elevatorMotor.setMode(RUN_USING_ENCODER);
                }
                break;
            case HOMING_WORM:
                rotateWorm(-100);
                if (wormLimitSwitch.isPressed()) {
                    homingState = COMPLETE;
                    elevatorMotor.setMode(STOP_AND_RESET_ENCODER);
                    elevatorMotor.setMode(RUN_USING_ENCODER);
                }
            case COMPLETE:
                normalPeriodArmState = AT_POS;
                break;
            case IDLE:
                break;
        }
    }

    /**
     * Displays debug information about the arm
     * @param telemetry The telemetry to display the information on
     * @param wormTelemetry Whether or not to display telemetry about the worm motor
     * @param elevatorTelemetry Whether or not to display telemetry about the elevator motor
     */
    public static void debug(@NonNull Telemetry telemetry, boolean wormTelemetry, boolean elevatorTelemetry) {
        telemetry.addData(
                "Worm Limit Switch Is Pressed",
                wormLimitSwitch.isPressed());
        telemetry.addData(
                "Elevator Limit Switch Is Pressed",
                elevatorLimitSwitch.isPressed());

        if (wormTelemetry) {
            telemetry.addData(
                    "Worm Motor Direction",
                    wormMotor.getDirection());
            telemetry.addData(
                    "Worm Motor Power",
                    wormMotor.getPower());
            telemetry.addData(
                    "Worm Motor Current (Amps)",
                    wormMotor.getCurrent(AMPS));
            telemetry.addData(
                    "Worm Current Pos",
                    wormMotor.getCurrentPosition());
            telemetry.addData(
                    "Worm Target Pos",
                    wormMotor.getTargetPosition());
        }

        if (elevatorTelemetry) {
            telemetry.addData(
                    "Elevator Motor Direction",
                    elevatorMotor.getDirection());
            telemetry.addData(
                    "Elevator Motor Power",
                    elevatorMotor.getPower());
            telemetry.addData(
                    "Elevator Motor Current (AMPS)",
                    elevatorMotor.getCurrent(AMPS));
            telemetry.addData(
                    "Elevator Motor Current Pos",
                    elevatorMotor.getCurrentPosition());
            telemetry.addData(
                    "Elevator Motor Target Pos",
                    elevatorMotor.getTargetPosition());
        }

        telemetry.addData(
                "Arm State - Normal Period",
                normalPeriodArmState);
        telemetry.addData(
                "Homing State",
                homingState);
        telemetry.addData(
                "Total Arm Current",
                elevatorMotor.getCurrent(AMPS) + wormMotor.getCurrent(AMPS));
    }

    /**
     * Extends the elevator to the provided position at the provided power
     * @param targetPos The position to move the elevator to
     * @param power The power to move to the target position at
     */
    private static void extendElevator(int targetPos, double power) {
        elevatorMotor.setTargetPosition(targetPos);
        elevatorMotor.setMode(RUN_TO_POSITION);
        elevatorMotor.setPower(power);
    }

    /**
     * Extends the elevator to the provided position at the power defined by DEFAULT_ELEVATOR_POWER
     * @param targetPos The position to move the elevator to
     */
    private static void extendElevator(int targetPos) {
        extendElevator(targetPos, DEFAULT_ELEVATOR_POWER);
    }

    /**
     * Rotates the worm to the provided position at the provided power
     * @param pos The position to move the worm to
     * @param power The power to move to the target position at
     */
    private static void rotateWorm(int pos, double power) {
        elevatorMotor.setTargetPosition(pos);
        elevatorMotor.setMode(RUN_TO_POSITION);
        elevatorMotor.setPower(power);
    }

    /**
     * Rotates the worm to the the provided position at the power defined by DEFAULT_WORM_POWER
     * @param targetPos The position to move the worm to
     */
    private static void rotateWorm(int targetPos) {
        rotateWorm(targetPos, DEFAULT_WORM_POWER);
    }

    /**
     * @return The current position of the elevator
     */
    public static int getElevatorPos() {
        return elevatorMotor.getCurrentPosition();
    }

    /**
     * @return The current position of the worm drive
     */
    public static int getWormPos() {
        return wormMotor.getCurrentPosition();
    }

    /**
     * @return The target position of the worm drive
     */
    public static int getWormTargetPos() {
        return wormMotor.getTargetPosition();
    }

    /**
     * @return The target position of the elevator
     */
    public static int getElevatorTargetPos() {
        return elevatorMotor.getTargetPosition();
    }

    /**
     * @return The state of the arm in the normal period
     */
    public static NormalPeriodArmState getNormalPeriodArmState() {
        return normalPeriodArmState;
    }

    /**
     * @return The homing state of the elevator
     */
    public static HomingState getHomingState() {
        return homingState;
    }
}
