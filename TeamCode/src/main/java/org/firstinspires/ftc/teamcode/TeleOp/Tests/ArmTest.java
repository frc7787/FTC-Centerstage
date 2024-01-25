package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import static org.firstinspires.ftc.teamcode.Properties.BOTTOM_EXT_POS;
import static org.firstinspires.ftc.teamcode.Properties.BOTTOM_ROT_POS;
import static org.firstinspires.ftc.teamcode.Properties.HIGH_EXT_POS;
import static org.firstinspires.ftc.teamcode.Properties.HIGH_ROT_POS;
import static org.firstinspires.ftc.teamcode.Properties.LOW_EXT_POS;
import static org.firstinspires.ftc.teamcode.Properties.LOW_ROT_POS;
import static org.firstinspires.ftc.teamcode.Properties.MED_EXT_POS;
import static org.firstinspires.ftc.teamcode.Properties.MED_ROT_POS;
import static org.firstinspires.ftc.teamcode.Properties.TOP_EXT_POS;
import static org.firstinspires.ftc.teamcode.Properties.TOP_ROT_POS;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;

@TeleOp(name = "Test - Arm", group = "Test")
@Disabled
public class ArmTest extends OpMode {
    Arm arm;

    Gamepad currentGamepadOne, prevGamepadOne, currentGamepadTwo, prevGamepadTwo;

    /**
     * Listens for arm commands. Note that this function calls arm.moveToPosition which updates the arm state.
     * @param currentGamepadOne The current state of gamepad1
     * @param prevGamepadOne The state of gamepad1 last loop iteration
     * @param currentGamepadTwo The current state of gamepad2
     * @param prevGamepadTwo The state of gamepad2 last loop iteration
     */
    void listenForNormalArmCommand(@NonNull Gamepad currentGamepadOne, @NonNull Gamepad prevGamepadOne, @NonNull Gamepad currentGamepadTwo, @NonNull Gamepad prevGamepadTwo) {
        if (currentGamepadOne.dpad_down && !prevGamepadOne.dpad_down || currentGamepadTwo.dpad_down && !prevGamepadTwo.dpad_down) {
            arm.setHoming();
        } else if (currentGamepadOne.cross && !prevGamepadOne.cross || currentGamepadTwo.cross && !prevGamepadTwo.cross) {
            arm.moveToPosition(BOTTOM_EXT_POS, BOTTOM_ROT_POS);
        } else if (currentGamepadOne.square && !prevGamepadOne.square || currentGamepadTwo.square && !prevGamepadTwo.square) {
            arm.moveToPosition(LOW_EXT_POS, LOW_ROT_POS);
        } else if (currentGamepadOne.circle && !prevGamepadOne.circle || currentGamepadTwo.circle && !prevGamepadTwo.circle) {
            arm.moveToPosition(MED_EXT_POS, MED_ROT_POS);
        } else if (currentGamepadOne.triangle && !prevGamepadOne.triangle || currentGamepadTwo.triangle && !prevGamepadTwo.triangle) {
            arm.moveToPosition(HIGH_EXT_POS, HIGH_ROT_POS);
        } else if (currentGamepadOne.options && !prevGamepadOne.options || currentGamepadTwo.options && !prevGamepadTwo.options) {
            arm.moveToPosition(TOP_EXT_POS, TOP_ROT_POS);
        }
    }


    @Override public void init() {
        currentGamepadOne = new Gamepad();
        prevGamepadOne    = new Gamepad();
        currentGamepadTwo = new Gamepad();
        prevGamepadTwo    = new Gamepad();

        arm = new Arm(hardwareMap);

        arm.init();
    }

    @Override public void loop() {
        prevGamepadOne.copy(currentGamepadOne);
        currentGamepadOne.copy(gamepad1);

        prevGamepadTwo.copy(currentGamepadOne);
        currentGamepadTwo.copy(gamepad2);

        // Update the arm state every loop iteration. Note: The arm state is also updated in the moveToPosition function which is called from the listenForNormalArmCommand
        arm.update();

        arm.debug(telemetry);

        // This function has an effect on the arm state
        listenForNormalArmCommand(
                currentGamepadOne,
                prevGamepadOne,
                currentGamepadTwo,
                prevGamepadTwo);

        telemetry.addLine("Press X, Square, Circle, Triangle, or Options to go to different heights");
        telemetry.addLine("Press Dpad Down to home");
        telemetry.update();
    }
}
