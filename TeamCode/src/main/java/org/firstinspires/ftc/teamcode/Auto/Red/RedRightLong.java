package org.firstinspires.ftc.teamcode.Auto.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropColor;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetector;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Red Right Long", group = "Red")
public class RedRightLong extends LinearOpMode {
    final PropDetector detector = new PropDetector(PropColor.RED);
    final Pose2d START_POS = new Pose2d(-36, -71.7, Math.toRadians(90));

    OpenCvCamera camera;

    PropLocation location;

    TrackingWheelLocalizer localizer;

    MecanumDriveBase drive;

    TrajectorySequence initial_path = drive.trajectorySequenceBuilder(START_POS)
            .lineTo(new Vector2d(12, -64))
            .build();

    TrajectorySequence to_backdrop_from_initial_position = drive.trajectorySequenceBuilder(START_POS)
            .lineTo(new Vector2d(39.00, -34.8))
            .build();

    TrajectorySequence to_pixel_stack_from_backdrop = drive.trajectorySequenceBuilder(new Pose2d(39, 34.8, Math.toRadians(0)))
            .lineTo(new Vector2d(38.79, -10.62))
            .lineTo(new Vector2d(-59.60, -10.83))
            .build();

    TrajectorySequence to_backdrop_from_pixel_stack = drive.trajectorySequenceBuilder(new Pose2d(-59.6, 10.83, Math.toRadians(0)))
            .lineTo(new Vector2d(38.79, -10.62))
            .lineTo(new Vector2d(39, -34.8))
            .build();

    int cameraMonitorViewId;

    @Override public void runOpMode() {
        drive = new MecanumDriveBase(hardwareMap);

        localizer = new TrackingWheelLocalizer(
                hardwareMap,
                new ArrayList<>(),
                new ArrayList<>());

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

        while (opModeIsActive() && !isStopRequested()) {
            // Drive forward so we don't clip the wall when we turn
            drive.followTrajectorySequence(initial_path);

            // Get the prop location
            location = detector.getLocation();

            // Place the pixel based on the prop location
            switch (location) {
                case LEFT:
                    drive.turn(0); // TODO Add Values
                    // Drop off pixel
                    drive.turn(0); // TODO Find value to turn away from pixel stacks
                case RIGHT:
                    drive.turn(0); // TODO Find value to to turn to the right spike strip
                    // Drop off pixel
                    drive.turn(0); // TODO Find the value to turn away from the pixel stacks
                case NONE:
                    // Drop off pixel
                    drive.turn(0); // TODO Find value to turn away from the pixel stacks
            }

            // Drive to the backdrop stack
            drive.followTrajectorySequence(to_backdrop_from_initial_position);

            // Drive to the pixel stack
            drive.followTrajectorySequence(to_pixel_stack_from_backdrop);

            // Intake pixel from stack


            // Drive to the back drop from pixel stack
            drive.followTrajectorySequence(to_backdrop_from_pixel_stack);

            // Place the pixels on the backdrop

            // For the future maybe try and go for a second time

        }

    }
}
