package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.*;

import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import static org.firstinspires.ftc.teamcode.Properties.*;

@TeleOp(name = "TeleOp drone testing", group = "Testing")
public class TeleOpDroneLaunch extends OpMode {
    private enum GamePeriod {
        NORMAL,
        ENDGAME
    }

    private GamePeriod gamePeriod = GamePeriod.ENDGAME;
    private Gamepad prevGamepad2, currentGamepad2;
    private boolean intaking   = false;

    private void listenForEndgameCommand() {
       if (gamepad2.left_bumper) {
           Arm.setTargetPos(0, LAUNCH_POS);
       }

       if (gamepad2.left_trigger > 0.9) {
           Auxiliaries.releaseLauncher();
       }
       if (gamepad2.right_trigger > 0.9) {
           Auxiliaries.resetLauncher();
       }

    }



    @Override public void init() {
        currentGamepad2 = new Gamepad();
        prevGamepad2    = new Gamepad();

        Intake.init(hardwareMap);
        Auxiliaries.init(hardwareMap);
        DriveBase.init(hardwareMap);
        Arm.init(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(AUTO);
        }
    }

    @Override public void loop() {
        //telemetry.addData("Intake trigger value", intakeTriggerValue);

        Arm.update(intaking);
        Arm.debug(telemetry);

        prevGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        double drive  = gamepad1.left_stick_y * -1.0; // Left stick y is inverted
        double strafe = gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

       DriveBase.driveManualRobotCentric(drive, strafe, turn);

       switch (gamePeriod) {
           case NORMAL:
               telemetry.addLine("Normal Period");

               // normalPeriodLoop();// not used in this opmode

               if (gamepad2.options && gamepad2.share) { gamePeriod = GamePeriod.ENDGAME; }
               break;
           case ENDGAME:
               telemetry.addLine("Endgame Period");

               listenForEndgameCommand();

               if (gamepad2.options && gamepad2.share) { gamePeriod = GamePeriod.NORMAL; }
               break;
       }

        telemetry.update();
    }
}