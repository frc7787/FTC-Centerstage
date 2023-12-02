package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GetRobotProperties;
import org.firstinspires.ftc.teamcode.Subsytems.Worm;

@TeleOp(name = "Test - Worm", group = "Test")
@Config
public class WormTest extends OpMode {

    private Worm worm;

    private enum WormState {
        TO_POSITION,
        AT_POSITION
    }

    public static WormState wormState = WormState.AT_POSITION;

    public static int BOTTOM_POS = GetRobotProperties.readInteger("BOTTOM_ROT_POSITION");
    public static int LOW_POS    = GetRobotProperties.readInteger("LOW_ROT_POSITION");
    public static int MED_POS    = GetRobotProperties.readInteger("MED_ROT_POSITION");
    public static int HIGH_POS   = GetRobotProperties.readInteger("HIGH_ROT_POSITION");
    public static int TOP_POS    = GetRobotProperties.readInteger("TOP_ROT_POSITION");

    @Override public void init() {
        worm = new Worm(hardwareMap);

        worm.init();
    }

    private void run_worm() {
        if (gamepad1.dpad_down || gamepad1.dpad_up) {
            worm.rotate(0);
        } else if (gamepad1.cross) {
            worm.rotate(BOTTOM_POS);
        } else if (gamepad1.square) {
            worm.rotate(LOW_POS);
        } else if (gamepad1.circle) {
            worm.rotate(MED_POS);
        } else if (gamepad1.triangle) {
            worm.rotate(HIGH_POS);
        } else if (gamepad1.options) {
            worm.rotate(TOP_POS);
        }
    }

    @Override public void loop() {
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
    }
}
