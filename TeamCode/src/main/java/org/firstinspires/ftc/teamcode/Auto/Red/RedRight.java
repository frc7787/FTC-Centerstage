package org.firstinspires.ftc.teamcode.Auto.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Red Right", group = "Red")
public class RedRight extends LinearOpMode {
    final PropDetector detector = new PropDetector(PropColor.RED);
    final Pose2d START_POS = new Pose2d(8.20, -63.00, Math.toRadians(90));

    AutoPath autoPath;

    OpenCvCamera camera;

    PropLocation location;

    TrackingWheelLocalizer localizer;

    MecanumDriveBase drive;

    Intake intake;

    TrajectorySequence inital_path = drive.trajectorySequenceBuilder(START_POS)
            .lineTo(new Vector2d(12.5, -56.7))
            .build();

    TrajectorySequence to_backdrop_from_initial_path = drive.trajectorySequenceBuilder(inital_path.end())
            .lineTo(BACKDROP_CENTER_POS_RED)
            .build();

    TrajectorySequence to_pixel_stack_from_backdrop_short = drive.trajectorySequenceBuilder(to_backdrop_from_initial_path.end())
            .lineTo(SHORT_PIXEL_STACK_RED)
            .build();

    TrajectorySequence to_pixel_stack_from_backdrop_long = drive.trajectorySequenceBuilder(to_backdrop_from_initial_path.end())
            .lineTo(new Vector2d(39.40, -11.60))
            .lineTo(LONG_PIXEL_STACK_RED)
            .build();

    TrajectorySequence to_backdrop_from_pixel_stack_short = drive.trajectorySequenceBuilder(to_pixel_stack_from_backdrop_long.end())
            .lineTo(BACKDROP_CENTER_POS_RED)
            .build();

    TrajectorySequence to_backdrop_from_pixel_stack_long = drive.trajectorySequenceBuilder(to_pixel_stack_from_backdrop_long.end())
            .lineTo(new Vector2d(39.40, -11.60))
            .lineTo(BACKDROP_CENTER_POS_RED)
            .build();

    TrajectorySequence park = drive.trajectorySequenceBuilder(to_backdrop_from_initial_path.end())
            .lineTo(new Vector2d(52, -35))
            .build();

    int cameraMonitorViewId;

    @Override public void runOpMode() {
        drive = new MecanumDriveBase(hardwareMap);
        intake = new Intake(hardwareMap);

        drive.init();

        localizer = new TrackingWheelLocalizer(
                hardwareMap,
                new ArrayList<>(),
                new ArrayList<>());

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

            telemetry.addData("Auto Path: ", autoPath);
        }

        while (opModeIsActive() && isStarted() && !isStopRequested()) {
            location = detector.getLocation();

            drive.followTrajectorySequence(inital_path);


            // All 0 values are temporary
            switch (location) {
                case LEFT:
                    drive.turn(0);
                    // Place pixel
                    drive.turn(0);
                case RIGHT:
                    drive.turn(0);
                    // Place pixel
                    drive.turn(0);
                case NONE:
                    // Place pixel
                    drive.turn(0);
            }

            drive.followTrajectorySequence(to_backdrop_from_initial_path);

            // Place pixel on backdrop

            switch (autoPath) {
                case SHORT:
                    // Drive to pixel stack
                    drive.followTrajectorySequence(to_pixel_stack_from_backdrop_short);

                    // Intake pixel
                    intake.intake(1000);

                    // Drive back to backdrop
                    drive.followTrajectorySequence(to_backdrop_from_pixel_stack_short);

                    // Place pixels on backdrop

                    // Park
                    drive.followTrajectorySequence(park);
                case LONG:
                    // Drive to pixel stack
                    drive.followTrajectorySequence(to_pixel_stack_from_backdrop_long);

                    // Intake pixel
                    intake.intake(1000);

                    // Drive back to backdrop
                    drive.followTrajectorySequence(to_backdrop_from_pixel_stack_long);

                    // Place pixels on backdrop

                    // Park
                    drive.followTrajectorySequence(park);
            }
        }
    }
}
