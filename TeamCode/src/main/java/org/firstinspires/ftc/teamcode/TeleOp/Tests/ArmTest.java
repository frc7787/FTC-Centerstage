package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import static org.firstinspires.ftc.teamcode.Subsytems.Arm.HomingState.COMPLETE;

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

    public void run_intake() {
        if (gamepad1.left_bumper) {
            arm.intake();
        } else if (gamepad1.right_bumper) {
            arm.outtake();
        }
    }

    private void home() {
        arm.home();

        if (arm.getHomingState() == COMPLETE) {
            arm.resetHomingState();
            armState = ArmState.AT_POSITION;
        }
    }

    public void run_arm() {
        if (gamepad1.dpad_down) {
            home();
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
        switch (armState) {
            case AT_POSITION:
                run_intake();
                run_arm();

                if (arm.is_busy()) { armState = ArmState.TO_POSITION; }
            case TO_POSITION:
                run_intake();
                run_arm();

                if (!arm.is_busy()) { armState = ArmState.AT_POSITION; }
        }
    }
}
