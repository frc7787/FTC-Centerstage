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

    public void openDoor() {
        leftDoor.setPosition(TRAY_DOOR_OPEN_POS);
        rightDoor.setPosition(TRAY_DOOR_CLOSED_POS);
    }

    public void closeDoor() {
        leftDoor.setPosition(TRAY_DOOR_CLOSED_POS);
        rightDoor.setPosition(TRAY_DOOR_CLOSED_POS);
    }

    public void raise_tray() {
        wrist.setPosition(TRAY_UP_WRIST_POS);
    }

    public void lower_tray() {
        wrist.setPosition(TRAY_DOWN_WRIST_POS);
    }
}
