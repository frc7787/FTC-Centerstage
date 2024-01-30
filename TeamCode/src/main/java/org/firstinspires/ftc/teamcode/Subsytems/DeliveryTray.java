package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.TRAY_DOOR_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.Properties.TRAY_DOOR_INTAKE_POS;
import static org.firstinspires.ftc.teamcode.Properties.TRAY_DOOR_OPEN_POS;
import static org.firstinspires.ftc.teamcode.Properties.TRAY_UP_WRIST_POS;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to contain the delivery tray subsystem
 */
public class DeliveryTray {
    Servo wrist, leftDoor, rightDoor;

    /**
     * @param hardwareMap The hardwareMap you are using, likely "hardwareMap"
     */
    public DeliveryTray(@NonNull HardwareMap hardwareMap) {
        wrist     = hardwareMap.get(Servo.class, "WristServo");
        leftDoor  = hardwareMap.get(Servo.class, "LeftDoorServo");
        rightDoor = hardwareMap.get(Servo.class, "RightDoorServo");
    }

    /**
     * Initializes the delivery tray by setting the wrist position to 0.
     */
    public void init() {
        wrist.setPosition(0);
    }

    /**
     * Moves the door of the tray to the position defined by TRAY_DOOR_OPEN_POS
     */
    public void openDoorToReleasePosition() {
        leftDoor.setPosition(TRAY_DOOR_OPEN_POS);
        rightDoor.setPosition(TRAY_DOOR_OPEN_POS);
    }

    /**
     * Moves the door of the tray to the position defined by TRAY_DOOR_INTAKE_POS
     */
    public void openDoorToIntakePos() {
        leftDoor.setPosition(TRAY_DOOR_INTAKE_POS);
        rightDoor.setPosition(TRAY_DOOR_INTAKE_POS);
    }

    /**
     * Moves the tray door servos to the position defined by the TRAY_DOOR_CLOSED_POS
     */
    public void closeDoor() {
        leftDoor.setPosition(TRAY_DOOR_CLOSED_POS);
        rightDoor.setPosition(TRAY_DOOR_CLOSED_POS);
    }

    /**
     * Raises the tray to the position defined by the TRAY_UP_WRIST_POS
     */
    public void raise_tray() {
        move_tray(TRAY_UP_WRIST_POS);
    }

    /**
     * Moves the tray to the supplied position
     * 
     * @param trayPos The position to move the tray to
     */
    public void move_tray(double trayPos) {
        wrist.setPosition(trayPos);
    }

    /**
     * Displays debug information about the delivery tray. Note to help improve loop time this function
     * DOES NOT call telemetry.update()
     * @param telemetry The telemetry to display the information on
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("Delivery Tray Debug");

        telemetry.addData("Left door direction", leftDoor.getDirection());
        telemetry.addData("Right door direction", rightDoor.getDirection());
        telemetry.addData("Wrist direction", wrist.getDirection());

        telemetry.addData("Left Door Target Position", leftDoor.getPosition());
        telemetry.addData("Right Door Target Position", rightDoor.getPosition());
        telemetry.addData("Wrist Target Position", wrist.getPosition());
    }
}
