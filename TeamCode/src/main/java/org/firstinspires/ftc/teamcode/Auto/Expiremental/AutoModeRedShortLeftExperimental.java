//package org.firstinspires.ftc.teamcode.Auto.Expiremental;
//
//import static org.firstinspires.ftc.teamcode.Auto.Utility.PropColor.RED;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Auto.Utility.AutoMode;
//import static org.firstinspires.ftc.teamcode.Auto.Utility.AutoMode.AutoPath.*;
//import static org.firstinspires.ftc.teamcode.Auto.Utility.AutoMode.AutoLocation.*;
//import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;
//
//@Autonomous(name = "Experimental - Auto Red Short", group = "Experimental")
//public class AutoModeRedShortLeftExperimental extends AutoMode {
//
//    @Override public void runOpMode() {
//        PropLocation location;
//
//        // Init auto. This sets an internal flag so you need to call this function or everything will break;
//        autoInit(RED, SHORT, LEFT);
//
//        // Wait for the driver to press start, if you have not called autoInit, will throw an error
//        waitForAutoStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            location = propDetector.getLocation();
//
//            driveBase.followTrajectorySequence(getSpikeStripTrajectory());
//
//            switch (location) {
//                case NONE:
//                    // Release preloaded pixel with intake.
//                case LEFT:
//                    driveBase.turn(-90); // Turn to face pixel stack
//                    // Release the preloaded pixel with intake
//                case RIGHT:
//                    driveBase.turn(90); // Turn to face pixel stack
//                    // Release the preloaded pixel with intake.
//                    driveBase.turn(180); // Turn to face the pixel stack
//            }
//
//            getPixelFromStackAndPlaceOnBackdrop(5);
//        }
//    }
//}
