package org.firstinspires.ftc.teamcode.Auto.Blue;

import static org.firstinspires.ftc.teamcode.Properties.BACKDROP_CENTER_POS_BLUE;
import static org.firstinspires.ftc.teamcode.Properties.LONG_PIXEL_STACK_BLUE;
import static org.firstinspires.ftc.teamcode.Properties.SHORT_PIXEL_STACK_BLUE;
import static org.firstinspires.ftc.teamcode.Properties.SHORT_PIXEL_STACK_RED;

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
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

// Name is reversed for the drivers perspective
@Autonomous(name = "Blue Right", group = "Blue")
public class BlueLeft extends LinearOpMode {
    final PropDetector detector = new PropDetector(PropColor.BLUE);
    final Pose2d START_POS = new Pose2d(7.7, 63, Math.toRadians(270));

    OpenCvCamera camera;

    PropLocation location;

    TrackingWheelLocalizer localizer;

    MecanumDriveBase drive;

    Intake intake;

    int cameraMonitorViewId;

    TrajectorySequence initial_path = drive.trajectorySequenceBuilder(START_POS)
            .lineTo(new Vector2d(-36.50, 57.00))
            .build();

    TrajectorySequence to_pixel_stack_from_inital_pos_short = drive.trajectorySequenceBuilder(initial_path.end())
            .lineTo(SHORT_PIXEL_STACK_BLUE)
            .build();

    TrajectorySequence to_pixel_stack_from_inital_pos_long = drive.trajectorySequenceBuilder(initial_path.end())
            .lineTo(new Vector2d(-57.00, 36.00))
            .lineTo(LONG_PIXEL_STACK_BLUE)
            .build();

    TrajectorySequence to_backdrop_from_pixel_stack_short = drive.trajectorySequenceBuilder(to_pixel_stack_from_inital_pos_short.end())
            .lineTo(BACKDROP_CENTER_POS_BLUE)
            .build();

    TrajectorySequence to_backdrop_from_pixel_stack_long = drive.trajectorySequenceBuilder(to_pixel_stack_from_inital_pos_long.end())
            .lineTo(new Vector2d(30.00, 11.60))
            .build();

    TrajectorySequence to_pixel_stack_from_backdrop_short = drive.trajectorySequenceBuilder(to_backdrop_from_pixel_stack_short.end())
            .lineTo(SHORT_PIXEL_STACK_BLUE)
            .build();

    TrajectorySequence to_pixel_stack_from_backdrop_long = drive.trajectorySequenceBuilder(to_backdrop_from_pixel_stack_long.end())
            .lineTo(new Vector2d(30.00, 11.60))
            .lineTo(LONG_PIXEL_STACK_BLUE)
            .build();

    TrajectorySequence park = drive.trajectorySequenceBuilder(to_backdrop_from_pixel_stack_long.end())
            .lineTo(new Vector2d(50.00, 36.70))
            .build();

    @Override public void runOpMode() {
        drive  = new MecanumDriveBase(hardwareMap);
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

        }
    }
}
