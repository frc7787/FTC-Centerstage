package org.firstinspires.ftc.teamcode.Auto.Expiremental;

import static org.firstinspires.ftc.teamcode.Auto.Utility.AutoMode.AutoPath.LONG;
import static org.firstinspires.ftc.teamcode.Auto.Utility.PropColor.RED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.Utility.AutoMode;
import static org.firstinspires.ftc.teamcode.Auto.Utility.AutoMode.AutoLocation.LEFT;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;

@Autonomous(name = "Auto - Experimental Run at your own risk!", group = "Experimental")
public class AutoModeRedLongLeftExperimental extends AutoMode {

    @Override public void runOpMode() {
        PropLocation location;

        autoInit(RED, LONG, LEFT); // Initialize Auto

        waitForAutoStart(); // Wait for the drive to press start

        while (opModeIsActive() && !isStopRequested()) {

            location = propDetector.getLocation(); // Get location

            // Move to spike strip; We do the same thing no matter what the location is, so putting it
            // in the switch statement is redundant
            driveBase.followTrajectorySequence(getSpikeStripTrajectory());

            switch (location) {
                case NONE:
                    // Release preloaded pixel with intake.
                case LEFT:
                    driveBase.turn(-90);
                    // Release the preloaded pixel with intake
                case RIGHT:
                    driveBase.turn(90);
                    // Release the preloaded pixel with intake.
                    driveBase.turn(180); // Turn to face the pixel stack
            }

            // Get the pixels and place them on the backdrop
            getPixelFromStackAndPlaceOnBackdrop(5);

            sleep(Long.MAX_VALUE); // Effectively sleep forever so we don't repeat the loop
        }
    }
}
