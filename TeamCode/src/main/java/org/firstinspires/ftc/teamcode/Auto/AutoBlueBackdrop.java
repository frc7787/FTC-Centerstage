package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropColor;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetector;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Blue - Backdrop", group = "Blue")
@Config
public class AutoBlueBackdrop extends LinearOpMode {
    Rect cropRectangle = new Rect(0, 120, 240, 120);

    PropDetector propDetector;
    PropLocation location;
    public static OpenCvCamera camera;

    MecanumDriveBase drive;



    @Override
    public void runOpMode() throws InterruptedException {


        propDetector = new PropDetector(PropColor.BLUE);
        drive        = new MecanumDriveBase(hardwareMap);


        drive.init();

        Pose2d startPose = new Pose2d(12, 66, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence toSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12.00, 36.00))
                .build();
        TrajectorySequence leftTurn = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence rightTurn = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(48.00, 36.00))
                .build();


        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId
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
        location = propDetector.getPropLocation();

        while (opModeIsActive()) {
            int leftCount    = 0;
            int rightCount   = 0;
            int noneCount    = 0;
            int centerCount  = 0;

            for (int i = 0; i <= 20;  i++) {
                switch (propDetector.getPropLocation()) {
                    case LEFT:
                        leftCount += 1;
                        break;
                    case RIGHT:
                        rightCount += 1;
                        break;
                    case NONE:
                        noneCount += 1;
                        break;
                    case CENTER:
                        centerCount += 1;
                        break;
                }
            }

            if (leftCount >= rightCount && leftCount >= noneCount) {
                location = PropLocation.LEFT;
            } else if (rightCount >= leftCount && rightCount >= noneCount) {
                location = PropLocation.RIGHT;
            } else {
                location = PropLocation.NONE;
            }

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();
            drive.followTrajectorySequence(toSpikeMark);
            switch (location) {
                case LEFT:
                    // Do the thing

                    sleep(1000);
                    drive.followTrajectorySequence(toBackdrop);
                    break;
                case RIGHT:
                    // Do the other thing

                    sleep(1000);

                    sleep(500);
                    drive.followTrajectorySequence(toBackdrop);
                    break;
                case CENTER:
                    // Do center or none
                    sleep(1000);

                    sleep(500);
                    drive.followTrajectorySequence(toBackdrop);
                    break;
                case NONE:
                    // Do center or none
                    sleep(1000);

                    sleep(500);
                    drive.followTrajectorySequence(toBackdrop);
                    break;
            } // end of switch location
            sleep(30000);
        } // end of while opmode is active
    }
}

