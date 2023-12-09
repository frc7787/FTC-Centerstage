package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.Properties.*;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;

@TeleOp(name = "Test - Arm", group = "Test")
@Disabled
public class ArmTest extends OpMode {

    Arm arm;

    enum ArmState {
        AT_POSITION,
        TO_POSITION,

    }

    ArmState armState = ArmState.AT_POSITION;

    Gamepad currentGamePad, prevGamePad;

    void listenForArmCommand(@NonNull Gamepad currentGamePad, @NonNull Gamepad prevGamePad) {
        if (currentGamePad.dpad_down && !prevGamePad.dpad_down) {
            arm.isHoming = true;
        } else if (currentGamePad.dpad_up && !prevGamePad.dpad_up) {
            arm.moveToPosition(MED_EXT_POS, 0);
        } else if (currentGamePad.cross && !prevGamePad.cross) {
            arm.moveToPosition(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (currentGamePad.square && !prevGamePad.square) {
            arm.moveToPosition(LOW_EXT_POS, LOW_ROT_POS);
        } else if (currentGamePad.circle && !prevGamePad.circle) {
            arm.moveToPosition(MED_EXT_POS, MED_ROT_POS);
        } else if (currentGamePad.triangle && !prevGamePad.triangle) {
            arm.moveToPosition(HIGH_EXT_POS, HIGH_ROT_POS);
        } else if (currentGamePad.options && !prevGamePad.options) {
            arm.moveToPosition(TOP_EXT_POS, TOP_ROT_POS);
        }
    }

    @Override public void init() {
        currentGamePad = new Gamepad();
        prevGamePad    = new Gamepad();

        arm = new Arm(hardwareMap);
    }

    @Override public void loop() {
        prevGamePad.copy(currentGamePad);
        currentGamePad.copy(gamepad2);

        telemetry.addLine("The light on the gamepad is dependent on the state of the arm.");
        telemetry.addLine("To Position = Blue");
        telemetry.addLine("At Position = Green");

        arm.debug(telemetry);

        telemetry.addData("Arm State", armState);

        if (gamepad1.left_bumper) { arm.intake(); }

        switch (armState) {
            case AT_POSITION:
                listenForArmCommand(currentGamePad, prevGamePad);

                if (arm.is_busy()) { armState = ArmState.TO_POSITION; }
                break;
            case TO_POSITION:
                listenForArmCommand(currentGamePad, prevGamePad);

                if (!arm.is_busy()) { armState = ArmState.AT_POSITION; }
                break;
        }
        telemetry.update();
    }
}
