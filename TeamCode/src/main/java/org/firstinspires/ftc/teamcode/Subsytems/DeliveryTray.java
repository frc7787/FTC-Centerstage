package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Properties.TRAY_DOOR_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.Properties.TRAY_DOOR_OPEN_POS;
import static org.firstinspires.ftc.teamcode.Properties.TRAY_DOWN_WRIST_POS;
import static org.firstinspires.ftc.teamcode.Properties.TRAY_UP_WRIST_POS;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryTray {
    Servo wrist, leftDoor, rightDoor;

    public DeliveryTray(@NonNull HardwareMap hardwareMap) {
        wrist     = hardwareMap.get(Servo.class, "WristServo");
        leftDoor  = hardwareMap.get(Servo.class, "LeftDoorServo");
        rightDoor = hardwareMap.get(Servo.class, "RightDoorServo");
    }

    /**
     * Opens the door of the tray
     */
    public void openDoor() {
        leftDoor.setPosition(TRAY_DOOR_OPEN_POS);
        rightDoor.setPosition(TRAY_DOOR_CLOSED_POS);
    }

    /**
     * Closes the door of the tray
     */
    public void closeDoor() {
        leftDoor.setPosition(TRAY_DOOR_CLOSED_POS);
        rightDoor.setPosition(TRAY_DOOR_CLOSED_POS);
    }

    /**
     * Raises the tray so that it is in paralell to the tray
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
}
