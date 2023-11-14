package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.*;

@TeleOp(name = "TeleOp 2023/2024 - Use This One", group = "Production")
public class TeleOpMain extends OpMode {

    private DriveBase driveBase;
    private Elevator elevator;
    private Intake intake;
    private Hanger hanger;
    private Launcher launcher;
    
    @Override public void init() {
        driveBase = new DriveBase(this);
        elevator  = new Elevator(this);
        intake    = new Intake(this);
        hanger    = new Hanger(this);
        launcher  = new Launcher(this);
    }

    @Override public void loop() {
        driveBase.run();
        elevator.run();
        intake.run();
        hanger.run();
        launcher.run();
    }
}