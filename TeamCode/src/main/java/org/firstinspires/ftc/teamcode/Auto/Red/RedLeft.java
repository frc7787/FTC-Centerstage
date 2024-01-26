package org.firstinspires.ftc.teamcode.Auto.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.AutoPath;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropColor;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetector;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Properties.SHORT_PIXEL_STACK_RED;
import static org.firstinspires.ftc.teamcode.Properties.LONG_PIXEL_STACK_RED;
import static org.firstinspires.ftc.teamcode.Properties.BACKDROP_CENTER_POS_RED;

import java.util.ArrayList;

// TODO Figure out if the roadrunner turn function turns BY the given angle or TO the given angle
// TODO if it is such that the turn function turns to the given angle the next two "todos" are not needed
// TODO Figure out the angles that we need to turn to reach the spike marks at the beginning of auto
// TODO Figure out the angles that we need to turn in order to face away from the backdrop
// TODO Make the "place pixel on backdrop".
@Autonomous(name = "Red Left", group = "Red")
@Disabled
public class RedLeft extends LinearOpMode {
    final PropDetector detector = new PropDetector(PropColor.RED);
    final Pose2d START_POS = new Pose2d(-31.50, -63, Math.toRadians(90));

    OpenCvCamera camera;

    PropLocation location;

    TrackingWheelLocalizer localizer;

    MecanumDriveBase drive;

    Intake intake;

    AutoPath autoPath = AutoPath.SHORT;

    int cameraMonitorViewId;

    // Confirmed to be correct
    TrajectorySequence initial_path = drive.trajectorySequenceBuilder(START_POS)
            .lineTo(new Vector2d(-36.50, -57.00))
            .build();


    // SHORT STUFF

    // Confirmed to be correct
    TrajectorySequence to_pixel_stack_from_inital_pos_short = drive.trajectorySequenceBuilder(initial_path.end())
            .lineTo(SHORT_PIXEL_STACK_RED)
            .build();

    // Confirmed to be correct
    TrajectorySequence to_backdrop_from_pixel_stack_short = drive.trajectorySequenceBuilder(to_pixel_stack_from_inital_pos_short.end())
            .lineTo(BACKDROP_CENTER_POS_RED)
            .build();

    // Confirmed to be correct
    TrajectorySequence to_pixel_stack_from_backdrop_short = drive.trajectorySequenceBuilder(to_backdrop_from_pixel_stack_short.end())
            .lineTo(SHORT_PIXEL_STACK_RED)
            .build();



    // LONG STUFF

    // Confirmed to be correct
    TrajectorySequence to_pixel_stack_from_inital_pos_long = drive.trajectorySequenceBuilder(initial_path.end())
            .lineTo(new Vector2d(-57.00, -36.00))
            .lineTo(LONG_PIXEL_STACK_RED)
            .build();

    // Confirmed to be correct
    TrajectorySequence to_backdrop_from_pixel_stack_long = drive.trajectorySequenceBuilder(to_pixel_stack_from_inital_pos_long.end())
            .lineTo(new Vector2d(35, -11.60))
            .lineTo(BACKDROP_CENTER_POS_RED)
            .build();

    // Confirmed to be correct
    TrajectorySequence to_pixel_stack_from_backdrop_long = drive.trajectorySequenceBuilder(to_backdrop_from_pixel_stack_long.end())
            .lineTo(new Vector2d(35, -11.60))
            .lineTo(LONG_PIXEL_STACK_RED)
            .build();



    // FOR BOTH

    // Confirmed to be correct
    TrajectorySequence park = drive.trajectorySequenceBuilder(to_backdrop_from_pixel_stack_long.end())
            .lineTo(new Vector2d(50.00, -11.60))
            .build();




    @Override public void runOpMode() {
        drive = new MecanumDriveBase(hardwareMap);
        intake = new Intake(hardwareMap);

        localizer = new TrackingWheelLocalizer(
                hardwareMap,
                new ArrayList<>(),
                new ArrayList<>());

        drive.init();
        localizer.init();

        localizer.setPoseEstimate(START_POS);

        cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        location = detector.getLocation();

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId
                );

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(detector);
            }

            @Override
            public void onError(int err_code) {
                telemetry.addLine("Camera failed to open with error code: " + err_code);
            }
        });

        while (opModeInInit() && !isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                autoPath = AutoPath.SHORT;
            } else if (gamepad1.circle) {
                autoPath = AutoPath.LONG;
            }

            telemetry.addData("Auto Path", autoPath);
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            location = detector.getLocation();

            drive.followTrajectorySequence(initial_path);

            // All zero values are temporary
            switch (location) {
                case LEFT:
                    drive.turn(0);
                    // Place pixel
                    drive.turn(0);
                case NONE:
                    // Place pixel
                    drive.turn(0);
                case RIGHT:
                    drive.turn(0);
                    // Place pixel
                    drive.turn(0);
            }

            switch (autoPath) {
                case SHORT:
                    // Drive to pixel stacks
                    drive.followTrajectorySequence(to_pixel_stack_from_inital_pos_short);

                    // Intake pixel
                    intake.intake(1000);

                    // Go to backdrop
                    drive.followTrajectorySequence(to_backdrop_from_pixel_stack_short);

                    // Place pixel on backdrop

                    // Drive back to pixel stack
                    drive.followTrajectorySequence(to_pixel_stack_from_backdrop_short);

                    // Intake pixels
                    intake.intake(1000);

                    // Drive to backdrop from pixel stack
                    drive.followTrajectorySequence(to_backdrop_from_pixel_stack_short);

                    // Place pixel on backdrop

                    // Park
                    drive.followTrajectorySequence(park);
                case LONG:
                    // Drive to pixel stack
                    drive.followTrajectorySequence(to_pixel_stack_from_inital_pos_long);

                    // Intake pixel
                    intake.intake(1000);

                    // Go to backdrop
                    drive.followTrajectorySequence(to_backdrop_from_pixel_stack_long);

                    // Place pixel on backdrop

                    // Drive back to pixel stack
                    drive.followTrajectorySequence(to_pixel_stack_from_backdrop_long);

                    // Intake pixel
                    intake.intake(1000);

                    // Drive back to backdrop
                    drive.followTrajectorySequence(to_backdrop_from_pixel_stack_long);

                    // Place pixel on backdrop

                    // Park
                    drive.followTrajectorySequence(park);
            }
        }
    }
}
