package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.*;
import org.firstinspires.ftc.teamcode.TeleOp.Utility.ElevatorPositions;

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
        //ElevatorPositions.readCSV();
        ElevatorPositions.updateElevatorConstants();

    }

    private void normal() {
        elevator.run();
        intake.run();
        intake.debug();
        driveBase.run();
    }

    private void endGame() {
        elevator.runEndGame(gamepad2);
        hanger.run();
        launcher.run();
        driveBase.run();
    }

    @Override public void loop() {

        // Enter Endgame
        if (gamepad2.left_trigger > 0.1 && gamepad2.right_trigger > 0.1) { endGame = true; }

        if (endGame) {
            telemetry.addLine("End Game");
            endGame();
        } else {
            telemetry.addLine("Normal Period");
            normal();
        }

        //elevator.debug();
        //telemetry.update();
    }
}