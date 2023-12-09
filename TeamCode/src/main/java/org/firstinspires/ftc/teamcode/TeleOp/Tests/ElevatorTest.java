package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsytems.Elevator;

import static org.firstinspires.ftc.teamcode.Properties.*;

import androidx.annotation.NonNull;

@TeleOp(name = "Test Elevator", group = "Test")
@Disabled
public class ElevatorTest extends OpMode {

    Elevator elevator;

    double maxCurrent = 0.0;

    enum ElevatorState {
        TO_POSITION,
        AT_POSITION
    }

    ElevatorState elevatorState = ElevatorState.AT_POSITION;

    Gamepad currentGamepad, prevGamepad;

    @Override public void init() {
        currentGamepad = new Gamepad();
        prevGamepad    = new Gamepad();

        elevator = new Elevator(hardwareMap);
        elevator.init();
    }

    void listenForElevatorCommand(@NonNull Gamepad currentGamepad, @NonNull Gamepad prevGamepad) {
        if (currentGamepad.dpad_up && !prevGamepad.dpad_up) {
            elevator.extend(MED_EXT_POS);
        } else if (currentGamepad.cross && !prevGamepad.cross) {
            elevator.extend(BOTTOM_EXT_POS);
        } else if (currentGamepad.square && !prevGamepad.square) {
            elevator.extend(LOW_EXT_POS);
        } else if (currentGamepad.circle && !prevGamepad.circle) {
            elevator.extend(MED_EXT_POS);
        } else if (currentGamepad.triangle && !prevGamepad.triangle) {
            elevator.extend(HIGH_EXT_POS);
        } else if (currentGamepad.options && !prevGamepad.options) {
            elevator.extend(TOP_EXT_POS);
        }
    }

    @Override public void loop() {
        prevGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad2);

        elevator.debug(telemetry);

        telemetry.addData("Elevator State", elevatorState);

        switch (elevatorState) {
            case AT_POSITION:
                listenForElevatorCommand(prevGamepad, currentGamepad);
                if (elevator.is_busy()) { elevatorState = ElevatorState.TO_POSITION; }
                break;
            case TO_POSITION:
                listenForElevatorCommand(prevGamepad, currentGamepad);
                if (!elevator.is_busy()) { elevatorState = ElevatorState.AT_POSITION; }
                break;
         }

         telemetry.update();
    }
}
