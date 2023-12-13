package org.firstinspires.ftc.teamcode.Auto;

import android.icu.lang.UProperty;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorBlue;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorRed;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RoadRunnerDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Auto Test - Blue", group = "Test")
public class AutoModeRed extends LinearOpMode {

    OpenCvCamera camera;

    @Override public void runOpMode() {

        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(
                hardwareMap,
                new ArrayList<>(),
                new ArrayList<>()
        );

        PropDetectorRed propDetector = new PropDetectorRed();

        Pose2d pose = new Pose2d(0,0, Math.toRadians(90));

        localizer.setPoseEstimate(pose);

        RoadRunnerDriveBase drive = new RoadRunnerDriveBase(hardwareMap);

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

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

        PropDetectorRed.PropLocation location;

        location = propDetector.getLocation();

        waitForStart();

        while(opModeIsActive()) {
            location = propDetector.getLocation();

            localizer.update();

            Pose2d currentPose = localizer.getPoseEstimate();

            if (location == PropDetectorRed.PropLocation.LEFT) {
                telemetry.addLine("Left");
            } else if (location == PropDetectorRed.PropLocation.RIGHT) {
                telemetry.addLine("Right");
            } else if (location == PropDetectorRed.PropLocation.NONE) {
                telemetry.addLine("None");
            }

            telemetry.update();
        }
    }
}
