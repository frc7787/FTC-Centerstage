package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.*;

@TeleOp(name = "TeleOp 2023/2024 - Use This One", group = "SDK")
public class Tyler extends OpMode {

    private DriveBase driveBase;
    private Elevator elevator;
    private Intake intake;
    
    @Override
    public void init() {
        driveBase = new DriveBase(this);
        elevator  = new Elevator(this);
        intake    = new Intake(this);
    }

    @Override
    public void loop() {
        driveBase.drive();
        elevator.run();
        elevator.debug();
        intake.run();
    }
}