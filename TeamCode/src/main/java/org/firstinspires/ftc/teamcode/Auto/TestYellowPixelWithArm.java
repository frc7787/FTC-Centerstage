package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.Utility.NormalPeriodArmState;

@Autonomous(name = "Test - Place Yellow Pixel With Arm")
public class TestYellowPixelWithArm extends LinearOpMode {

    enum PlacingState {
        START,
        MOVING_TO_POS,
        PLACING,
        PLACED
    }

    int wormTargetPos     = 725;
    int elevatorTargetPos = 2562;

    enum POSITION {
        LEFT,
        RIGHT
    }

    PlacingState placingState = PlacingState.START;
    POSITION position = POSITION.LEFT;

    @Override public void runOpMode() {
        Arm.init(hardwareMap);

        while (opModeInInit()) {
            telemetry.addLine("Press left bumper for left placement, and right for right.");
            telemetry.addData("Current Placement", position);

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                position = POSITION.LEFT;
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                position = POSITION.RIGHT;
            }

            telemetry.update();

            sleep(100);
        }

        waitForStart();

        if (isStopRequested()) { return; }

        while (placingState != PlacingState.PLACED) {
            Arm.update(false);

            telemetry.addData("Placing State", placingState);
            telemetry.update();

            switch (placingState) {
                case START:
                    Arm.setTargetPos(elevatorTargetPos, wormTargetPos);

                    placingState = PlacingState.MOVING_TO_POS;
                    break;
                case MOVING_TO_POS:
                    if (Arm.getNormalPeriodArmState() == NormalPeriodArmState.AT_POS) {
                        placingState = PlacingState.PLACING;
                    }
                    break;
                case PLACING:
                    switch (position) {
                        case LEFT:
                            Arm.openDeliveryTrayDoorLeft(1.0);
                        case RIGHT:
                            Arm.openDeliveryTrayDoorLeft(1.0);
                    }

                    sleep(1000);
                    placingState = PlacingState.PLACED;
                    break;
                case PLACED:
                    break;
            }
        }


    }
}
