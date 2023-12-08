package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Properties.*;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;

@TeleOp(name = "Test - Arm", group = "Test")
@Disabled
public class ArmTest extends OpMode {

    private Arm arm;

    private enum ArmState {
        AT_POSITION,
        TO_POSITION,

    }

    public static ArmState armState = ArmState.AT_POSITION;

    public void listenForArmCommand() {
        if (gamepad1.dpad_down) {
            arm.isHoming = true;
        } else if (gamepad1.dpad_up) {
            arm.moveToPosition(MED_EXT_POS, 0);
        } else if (gamepad1.cross) {
            arm.moveToPosition(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (gamepad1.square) {
            arm.moveToPosition(LOW_EXT_POS, LOW_ROT_POS);
        } else if (gamepad1.circle) {
            arm.moveToPosition(MED_EXT_POS, MED_ROT_POS);
        } else if (gamepad1.triangle) {
            arm.moveToPosition(HIGH_EXT_POS, HIGH_ROT_POS);
        } else if (gamepad1.options) {
            arm.moveToPosition(TOP_EXT_POS, TOP_ROT_POS);
        }
    }

    @Override public void init() {
        arm = new Arm(hardwareMap);
    }

    @Override public void loop() {
        arm.debug(telemetry);

        telemetry.addData("Arm State", armState);

        if (gamepad1.left_bumper) { arm.intake(); }

        switch (armState) {
            case AT_POSITION:
                listenForArmCommand();

                if (arm.is_busy()) { armState = ArmState.TO_POSITION; }
                break;
            case TO_POSITION:
                listenForArmCommand();

                if (!arm.is_busy()) { armState = ArmState.AT_POSITION; }
                break;
        }
        telemetry.update();
    }
}
