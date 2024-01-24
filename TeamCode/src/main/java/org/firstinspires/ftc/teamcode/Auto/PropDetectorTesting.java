package org.firstinspires.ftc.teamcode.Auto;

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

import java.util.ArrayList;

@Autonomous(name = "PROP DETECTOR TESTING")
public class PropDetectorTesting extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;
    public static OpenCvCamera camera;

    MecanumDriveBase drive;

    Intake intake;


    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetector(PropColor.BLUE);
        drive  = new MecanumDriveBase(hardwareMap);
        intake = new Intake(hardwareMap);

        drive.init();

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


        waitForStart();

        // Pls do not delete this
        location = propDetector.getLocation();

        while (opModeIsActive()) {
            location = propDetector.getLocation();


            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

            switch (location) {
                case LEFT:
                    // THIS IS ACTUALLY NONE
                    break;
                case NONE:
                    // THIS IS ACTUALLY LEFT
                    drive.turn(3.14 * 0.10);
                    break;
                case RIGHT:
                    // ACTUALLY RIGHT
                    drive.turn(-3.14 * 0.10);
                    break;
            }

            sleep(99999999);
        }
    }
}