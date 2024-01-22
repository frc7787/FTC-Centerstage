package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.TRAY_DOOR_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.Properties.TRAY_DOOR_OPEN_POS;
import static org.firstinspires.ftc.teamcode.Properties.TRAY_DOWN_WRIST_POS;
import static org.firstinspires.ftc.teamcode.Properties.TRAY_UP_WRIST_POS;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DeliveryTray {
    Servo wrist, leftDoor, rightDoor;

    public DeliveryTray(@NonNull HardwareMap hardwareMap) {
        wrist     = hardwareMap.get(Servo.class, "WristServo");
        leftDoor  = hardwareMap.get(Servo.class, "LeftDoorServo");
        rightDoor = hardwareMap.get(Servo.class, "RightDoorServo");
    }

    public void init() {
    }

    /**
     * Opens the door of the tray
     */
    public void openDoor() {
        leftDoor.setPosition(TRAY_DOOR_OPEN_POS);
        rightDoor.setPosition(TRAY_DOOR_OPEN_POS);
    }

    /**
     * Closes the door of the tray
     */
    public void closeDoor() {
        leftDoor.setPosition(TRAY_DOOR_CLOSED_POS);
        rightDoor.setPosition(TRAY_DOOR_CLOSED_POS);
    }

    /**
     * Raises the tray so that it is in parallel to the elevator
     */
    public void raise_tray() {
        wrist.setPosition(TRAY_UP_WRIST_POS);
    }

    /**
     * Moves the tray to the supplied position
     * 
     * @param pos: The position to move the tray to
     */
    public void move_tray(double pos) {
        wrist.setPosition(pos);
    }

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
