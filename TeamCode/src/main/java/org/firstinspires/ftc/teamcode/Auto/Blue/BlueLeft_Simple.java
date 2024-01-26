package org.firstinspires.ftc.teamcode.Auto.Blue;

import static org.firstinspires.ftc.teamcode.Properties.BACKDROP_CENTER_POS_BLUE;
import static org.firstinspires.ftc.teamcode.Properties.LONG_PIXEL_STACK_BLUE;
import static org.firstinspires.ftc.teamcode.Properties.SHORT_PIXEL_STACK_BLUE;
import static org.firstinspires.ftc.teamcode.Properties.SHORT_PIXEL_STACK_RED;

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

import java.util.ArrayList;

// Name is reversed for the drivers perspective
@Autonomous(name = "Blue right Simple", group = "Blue")
@Disabled
public class BlueLeft_Simple extends LinearOpMode {
    final PropDetector detector = new PropDetector(PropColor.BLUE);
    final Pose2d START_POS = new Pose2d(-31.7, 63.3, Math.toRadians(270));
    AutoPath path = AutoPath.SHORT;

    OpenCvCamera camera;

    PropLocation location;

    int cameraMonitorViewId;



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        drive.init();

        drive.setPoseEstimate(START_POS);
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

        waitForStart();

        if (!isStopRequested()) {
            TrajectorySequence initial_path = drive.trajectorySequenceBuilder(START_POS)
                    .lineToConstantHeading(new Vector2d(-36.50, 57.00))
                    .build();
            location = detector.getLocation();

            drive.followTrajectorySequence(initial_path);

            // All of the cases follow the following steps
            //
            // 1. Turn to face the spike strip
            // 2. Place the pixel on the spike strip
            // 3. Turn to face away from the pixel stacks
            switch (location) {// drop purple pixel in correct spot
                case LEFT:
                    TrajectorySequence purple_left = drive.trajectorySequenceBuilder(START_POS)
                            .lineTo(new Vector2d(-36.50, 57.00))
                            .build();
                    drive.turn(Math.toRadians(20));
                    // Place pixel on spike strip
                    drive.turn(0);
                    break;

                case NONE:
                    // Place pixel on spike strip
                    drive.turn(0);
                    break;

                case RIGHT:
                    drive.turn(Math.toRadians(-20));
                    // Place pixel on spike strip
                    drive.turn(0);
                    break;

            }
            switch (location) {// drop yellow pixel in correct spot
                case LEFT:
                    drive.turn(Math.toRadians(20));
                    // Place pixel on spike strip
                    drive.turn(0);
                    break;

                case NONE:
                    // Place pixel on spike strip
                    drive.turn(0);
                    break;

                case RIGHT:
                    drive.turn(Math.toRadians(-20));
                    // Place pixel on spike strip
                    drive.turn(0);
                    break;

            }
            switch (location) {//park
                case LEFT:
                    drive.turn(Math.toRadians(20));
                    // Place pixel on spike strip
                    drive.turn(0);
                    break;

                case NONE:
                    // Place pixel on spike strip
                    drive.turn(0);
                    break;

                case RIGHT:
                    drive.turn(Math.toRadians(-20));
                    // Place pixel on spike strip
                    drive.turn(0);
                    break;

            }


        }
    }
}
