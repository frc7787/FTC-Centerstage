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
    private boolean endGame = false;
    
    @Override public void init() {
        driveBase = new DriveBase(this);
        elevator  = new Elevator(this);
        intake    = new Intake(this);
        hanger    = new Hanger(this);
        launcher  = new Launcher(this);
    }

    @Override public void loop() {

        // Transition to endgame
        if (endGame && gamepad1.touchpad && gamepad1.left_trigger == 1 || gamepad2.touchpad && gamepad2.left_trigger == 1) { endGame = false; }

        // Leave endgame if we want
        if (gamepad1.touchpad && gamepad1.left_trigger == 1 || gamepad2.touchpad && gamepad2.left_trigger == 1) { endGame = true; }

        if (endGame) {
            elevator.runEndGame(gamepad2);
            hanger.run();
            //launcher.run();
        } else {
            elevator.run();
            intake.run();
        }

        driveBase.run();

        //launcher.run();
    }
}