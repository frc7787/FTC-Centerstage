package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import static org.firstinspires.ftc.teamcode.Subsytems.Arm.HomingState.COMPLETE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GetRobotProperties;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;

@TeleOp(name = "Test - Arm", group = "Test")
@Config
public class ArmTest extends OpMode {

    private Arm arm;

    private enum ArmState {
        AT_POSITION,
        TO_POSITION,

    }

    public static ArmState armState = ArmState.AT_POSITION;

    public static double INTAKE_SPEED  = GetRobotProperties.readDouble("INTAKE_SPEED");
    public static double OUTTAKE_SPEED = GetRobotProperties.readDouble("OUTTAKE_SPEED");

    public static int BOTTOM_EXT_POS = GetRobotProperties.readInteger("BOTTOM_ROT_POSITION");
    public static int LOW_EXT_POS    = GetRobotProperties.readInteger("LOW_ROT_POSITION");
    public static int MED_EXT_POS    = GetRobotProperties.readInteger("MED_ROT_POSITION");
    public static int HIGH_EXT_POS   = GetRobotProperties.readInteger("HIGH_ROT_POSITION");
    public static int TOP_EXT_POS    = GetRobotProperties.readInteger("TOP_ROT_POSITION");

    public static int BOTTOM_ROT_POS = GetRobotProperties.readInteger("BOTTOM_EXTEND_POSITION");
    public static int LOW_ROT_POS    = GetRobotProperties.readInteger("LOW_EXTEND_POSITION");
    public static int MED_ROT_POS    = GetRobotProperties.readInteger("MED_EXTEND_POSITION");
    public static int HIGH_ROT_POS   = GetRobotProperties.readInteger("HIGH_EXTEND_POSITION");
    public static int TOP_ROT_POS    = GetRobotProperties.readInteger("TOP_EXTEND_POSITION");

    public void run_intake() {
        if (gamepad1.left_bumper) {
            arm.intakeTest(INTAKE_SPEED);
        } else if (gamepad1.right_bumper) {
            arm.outtakeTest(OUTTAKE_SPEED);
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
