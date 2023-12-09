package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.Properties.*;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Subsytems.Worm;

@TeleOp(name = "Test - Worm", group = "Test")
@Disabled
public class WormTest extends OpMode {

    Worm worm;

    enum WormState {
        TO_POSITION,
        AT_POSITION
    }

   WormState wormState = WormState.AT_POSITION;

    Gamepad currentGamepad, prevGamepad;

    @Override public void init() {
        currentGamepad = new Gamepad();
        prevGamepad    = new Gamepad();

        worm = new Worm(hardwareMap);

        worm.init();
    }

    void listenForWormCommand(@NonNull Gamepad currentGamepad, @NonNull Gamepad prevGamepad) {
        if (currentGamepad.left_bumper || currentGamepad.right_bumper && !prevGamepad.left_bumper || !prevGamepad.right_bumper) {
            worm.rotate(0);
        } else if (currentGamepad.cross && !prevGamepad.cross) {
            worm.rotate(BOTTOM_ROT_POS);
        } else if (currentGamepad.square && !prevGamepad.square) {
            worm.rotate(LOW_ROT_POS);
        } else if (currentGamepad.circle && ! prevGamepad.circle) {
            worm.rotate(MED_ROT_POS);
        } else if (currentGamepad.triangle && !prevGamepad.triangle) {
            worm.rotate(HIGH_ROT_POS);
        } else if (currentGamepad.options && !prevGamepad.options) {
            worm.rotate(TOP_ROT_POS);
        }
    }

    @Override public void loop() {
        prevGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        worm.debug(telemetry);

        telemetry.addData("Worm State", wormState);

        switch (wormState) {
            case AT_POSITION:
                listenForWormCommand(currentGamepad, prevGamepad);
                if (worm.is_busy()) { wormState = WormState.TO_POSITION; }
                break;
            case TO_POSITION:
                listenForWormCommand(currentGamepad, prevGamepad);
                if (!worm.is_busy()) { wormState = WormState.AT_POSITION; }
                break;
        }

        telemetry.update();
    }
}
