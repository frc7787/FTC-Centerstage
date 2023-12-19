package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorBlue;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorRed;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RoadRunnerDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Auto Mode Blue Long- USE THIS!", group = "Test")
public class AutoModeBlueLong extends LinearOpMode {

    OpenCvCamera camera;

    @Override public void runOpMode() {
        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        PropDetectorBlue propDetector = new PropDetectorBlue();

        PropDetectorBlue.PropLocation location;

        location = propDetector.getLocation();

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId
                );

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(propDetector);
            }

            @Override public void onError(int errorCode) {
                telemetry.addData("Failed to open camera due to error code", errorCode);
                telemetry.update();
            }
        });

        // Roadrunner stuff
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(
                hardwareMap,
                new ArrayList<>(),
                new ArrayList<>()
        );

        Pose2d pose = new Pose2d(-36.04, 71.71, Math.toRadians(270));

        localizer.setPoseEstimate(pose);

        RoadRunnerDriveBase drive = new RoadRunnerDriveBase(hardwareMap);


        // Our drive sequences
        TrajectorySequence left = drive.trajectorySequenceBuilder(pose)
                .splineTo(new Vector2d(-47.59, 35.75), Math.toRadians(270.00))
                .build();



        TrajectorySequence right = drive.trajectorySequenceBuilder(pose)
                .lineTo(new Vector2d(-47.45, 35.75))
                .lineTo(new Vector2d(56.98, 36.04))
                .build();


        waitForStart();

        while(opModeIsActive()) {
            location = propDetector.getLocation();

            telemetry.addData("LOCATION: ", location);
            telemetry.update();

            localizer.update();

            Pose2d currentPose = localizer.getPoseEstimate();


            if (location == PropDetectorBlue.PropLocation.LEFT) {
                telemetry.addLine("RUNNING LEFT");
                //drive.followTrajectorySequence(left);
            } else if (location == PropDetectorBlue.PropLocation.RIGHT) {
                telemetry.addLine("RUNNING RIGHT");
                //drive.followTrajectorySequence(right);
            } else if (location == PropDetectorBlue.PropLocation.NONE) {
                telemetry.addLine("RUNNING NONE");
            }

            //telemetry.update();

            //sleep(1000000);
        }
    }
}