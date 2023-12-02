package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GetRobotProperties;
import org.firstinspires.ftc.teamcode.Subsytems.Elevator;

@TeleOp(name = "Test Elevator", group = "Test")
@Config
public class ElevatorTest extends OpMode {

    private static Elevator elevator;

    private enum ElevatorState {
        TO_POSITION,
        AT_POSITION
    }

    public static ElevatorState elevatorState = ElevatorState.AT_POSITION;

    public static int BOTTOM_POS = GetRobotProperties.readInteger("BOTTOM_EXTEND_POSITION");
    public static int LOW_POS    = GetRobotProperties.readInteger("LOW_EXTEND_POSITION");
    public static int MED_POS    = GetRobotProperties.readInteger("MED_EXTEND_POSITION");
    public static int HIGH_POS   = GetRobotProperties.readInteger("HIGH_EXTEND_POSITION");
    public static int TOP_POS    = GetRobotProperties.readInteger("TOP_EXTEND_POSITION");

    @Override public void init() {
        elevator = new Elevator(hardwareMap);

        elevator.init();
    }

    private void run_elevator() {
        if (gamepad1.dpad_down) {
            elevator.extend(MED_POS);
        } else if (gamepad1.cross) {
            elevator.extend(BOTTOM_POS);
        } else if (gamepad1.square) {
            elevator.extend(LOW_POS);
        } else if (gamepad1.circle) {
            elevator.extend(MED_POS);
        } else if (gamepad1.triangle) {
            elevator.extend(HIGH_POS);
        } else if (gamepad1.options) {
            elevator.extend(TOP_POS);
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
