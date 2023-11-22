package org.firstinspires.ftc.teamcode.TeleOp.Utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;

@TeleOp(name = "Zero Elevator", group = "Utility")
public class ZeroElevator extends OpMode {

    public Arm arm;

    @Override public void init() {
        arm = new Arm(hardwareMap);
        arm.init();
    }

    @Override public void loop() {
        if (!arm.extensionLimitSwitchIsPressed()) { arm.powerElevator(-0.5); }
        if (!arm.rotationLimitSwitchIsPressed())  { arm.powerWorm(-0.5);     }
    }

}
