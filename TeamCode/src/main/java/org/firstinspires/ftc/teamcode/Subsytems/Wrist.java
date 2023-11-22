package org.firstinspires.ftc.teamcode.Subsytems;

import static org.firstinspires.ftc.teamcode.Constants.WRIST_LEVEL_POSITION;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

final class Wrist {

    ServoImplEx wristServo;

    public Wrist(@NonNull HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(ServoImplEx.class, "wS");
    }

    /**
     * Initializes the wrist object
     */
    public void init() {
        wristServo.scaleRange(0.25, 0.8);
        wristServo.setPosition(WRIST_LEVEL_POSITION);
    }

    /**
     * Levels The Wrist
     */
    public void level() { wristServo.setPosition(WRIST_LEVEL_POSITION); }

    /**
     * Sets the wrist to a position
     * @param position The Position to set the servo to, between 0 and 1
     */
    public void setPosition(double position) { wristServo.setPosition(position); }

    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addData("Wrist Position", wristServo.getPosition());
        telemetry.addData("Wrist Direction", wristServo.getDirection());
        telemetry.addData("Wrist PWM Range", wristServo.getPwmRange());
    }
}
