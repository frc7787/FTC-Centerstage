package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Elevator;

@TeleOp(name = "Test - Elevator", group = "Test")
public class ElevatorTest extends OpMode {

    public Elevator elevator;

    @Override public void init() { elevator = new Elevator(this); }

    @Override public void loop() {
        elevator.run();
        elevator.debug();
    }
}
