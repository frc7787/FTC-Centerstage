package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Properties.*;
import org.firstinspires.ftc.teamcode.Subsytems.Worm;

@TeleOp(name = "Test - Worm", group = "Test")
@Disabled
public class WormTest extends OpMode {

    private Worm worm;

    private enum WormState {
        TO_POSITION,
        AT_POSITION
    }

    public static WormState wormState = WormState.AT_POSITION;

    @Override public void init() {
        worm = new Worm(hardwareMap);

        worm.init();
    }

    private void run_worm() {
        if (gamepad1.dpad_down || gamepad1.dpad_up) {
            worm.rotate(0);
        } else if (gamepad1.cross) {
            worm.rotate(BOTTOM_ROT_POS);
        } else if (gamepad1.square) {
            worm.rotate(LOW_ROT_POS);
        } else if (gamepad1.circle) {
            worm.rotate(MED_ROT_POS);
        } else if (gamepad1.triangle) {
            worm.rotate(HIGH_ROT_POS);
        } else if (gamepad1.options) {
            worm.rotate(TOP_ROT_POS);
        }
    }

    @Override public void loop() {
        worm.debug(telemetry);

        telemetry.addData("Worm State", wormState);

        switch (wormState) {
            case AT_POSITION:
                run_worm();
                if (worm.is_busy()) { wormState = WormState.TO_POSITION; }
                break;
            case TO_POSITION:
                run_worm();
                if (!worm.is_busy()) { wormState = WormState.AT_POSITION; }
                break;
        }

        telemetry.update();
    }
}
