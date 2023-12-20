package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorRed;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RoadRunnerDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Auto Mode Red Short", group = "Test")
public class AutoModeRedShort extends LinearOpMode {

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

        PropLocation location;

        location = propDetector.getLocation();


        // Our drive sequences





        waitForStart();

        while(opModeIsActive()) {
            location = propDetector.getLocation();

            telemetry.addData("LOCATION: ", location);
            telemetry.update();

            localizer.update();

            Pose2d currentPose = localizer.getPoseEstimate();


            if (location == PropLocation.LEFT) {
                drive.turn(Math.toRadians(-45));
            } else if (location == PropLocation.RIGHT) {
                drive.turn(Math.toRadians(45));
            } else if (location == PropLocation.NONE) {

            }

            sleep(1000000);
        }
    }
}