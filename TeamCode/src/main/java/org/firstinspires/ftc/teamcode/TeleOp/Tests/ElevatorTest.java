package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytems.Elevator;

import static org.firstinspires.ftc.teamcode.Properties.*;

@TeleOp(name = "Test Elevator", group = "Test")
@Disabled
public class ElevatorTest extends OpMode {

    private static Elevator elevator;

    private enum ElevatorState {
        TO_POSITION,
        AT_POSITION
    }

    public static ElevatorState elevatorState = ElevatorState.AT_POSITION;

    @Override public void init() {
        elevator = new Elevator(hardwareMap);

        elevator.init();
    }

    private void run_elevator() {
        if (gamepad1.dpad_down) {
            elevator.extend(MED_EXT_POS);
        } else if (gamepad1.cross) {
            elevator.extend(BOTTOM_EXT_POS);
        } else if (gamepad1.square) {
            elevator.extend(LOW_EXT_POS);
        } else if (gamepad1.circle) {
            elevator.extend(MED_EXT_POS);
        } else if (gamepad1.triangle) {
            elevator.extend(HIGH_EXT_POS);
        } else if (gamepad1.options) {
            elevator.extend(TOP_EXT_POS);
        }
    }

    @Override public void loop() {
        switch (elevatorState) {
            case AT_POSITION:
                run_elevator();
                if (elevator.is_busy()) { elevatorState = ElevatorState.TO_POSITION; }
                break;
            case TO_POSITION:
                run_elevator();
                if (!elevator.is_busy()) { elevatorState = ElevatorState.AT_POSITION; }
                break;
         }
    }
}
