package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.Auxiliaries;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Blue - Test1", group = "Blue")
@Config
public class AutoTest1 extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;

    OpenCvCamera camera;
    MecanumDriveBase drive;

    int cameraMonitorViewId;

    int leftCount   = 0;
    int rightCount  = 0;
    int noneCount   = 0;
    int centerCount = 0;

    int spikeStripPixelPlacementSleep = 1000;
    int backdropPixelPlacementSleep   = 800;

    TrajectorySequence toSpikeLeft, toSpikeCenter, toSpikeRight;
    TrajectorySequence toBackdropLeft, toBackdropCenter, toBackdropRight;
    TrajectorySequence toParkLeft, toParkCenter, toParkRight;
    TrajectorySequence testC1, testC2, testC3, testR1, testR2, testR3;

    @Override public void runOpMode() throws InterruptedException {
        propDetector = new PropDetector(PropColor.BLUE);
        drive        = new MecanumDriveBase(hardwareMap);

        drive.init();

        Pose2d startPose = new Pose2d(11, 63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Arm.init(hardwareMap);
        Auxiliaries.init(hardwareMap);

        Arm.update(false);

        toSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, 61))
                .strafeTo(new Vector2d(33, 61))
                .lineTo(new Vector2d(33, 40))
                .strafeTo(new Vector2d(20, 40))
                .strafeTo(new Vector2d(33, 40))
                .lineTo(new Vector2d(33, 11))
                .strafeTo(new Vector2d(16, 11))
                .build();

        toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(45, 11))
                .strafeTo(new Vector2d(45, 35))
                .lineTo(new Vector2d(50, 35))
                .build();

        toParkLeft = drive.trajectorySequenceBuilder(toBackdropLeft.end())
                .lineTo(new Vector2d(47, 35))
                .strafeTo(new Vector2d(47, 63))
                .build();

        toSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, 60))
                .strafeTo(new Vector2d(28, 60))
                .lineTo(new Vector2d(28, 28))
                .strafeTo(new Vector2d(14, 28))
                .lineTo(new Vector2d(14, 10))
                .strafeTo(new Vector2d(6, 10))
                .build();

        toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(45, 10))
                .strafeTo(new Vector2d(45, 24))
                .lineTo(new Vector2d(50, 24))
                .build();

        toParkCenter = drive.trajectorySequenceBuilder(toBackdropCenter.end())
                .lineTo(new Vector2d(49, 28))
                .strafeTo(new Vector2d(49, 63))
                .build();

        toSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, 30))
                .strafeTo(new Vector2d(7, 30))
                .strafeTo(new Vector2d(11, 30))
                .lineTo(new Vector2d(11, 13))
                .strafeTo(new Vector2d(-6, 13))
                .build();

        toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(45, 13))
                .strafeTo(new Vector2d(45, 21))
                .lineTo(new Vector2d(50, 21))
                .build();

        toParkRight = drive.trajectorySequenceBuilder(toBackdropRight.end())
                .lineTo(new Vector2d(47, 21))
                .strafeTo(new Vector2d(47, 63))
                .build();
        testC1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(6, 27))
                .build();
        testC2 = drive.trajectorySequenceBuilder(testC1.end())
                .lineToLinearHeading(new Pose2d(45, 24, Math.toRadians(0)))
                .lineTo(new Vector2d(50, 24))
                .build();
        testC3 = drive.trajectorySequenceBuilder(testC2.end())
                .lineToConstantHeading(new Vector2d(50, 36))
                .lineToConstantHeading(new Vector2d(15, 36))
                .build();
        testR1 = drive.trajectorySequenceBuilder(startPose)
                .turn(2)
                .build();
        testR1 = drive.trajectorySequenceBuilder(testR1.end())
                .turn(2)
                .build();

        cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

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

        Arm.rotateWorm(1400);

        waitForStart();

        if (isStopRequested()) { return; }

        for (int i = 0; i <= 20;  i++) {
            switch (propDetector.getPropLocation()) {
                case LEFT:
                    leftCount += 1;
                    break;
                case RIGHT:
                    rightCount += 1;
                    break;
                case CENTER:
                    centerCount += 1;
                case NONE:
                    noneCount += 1;
                    break;
            }
        }

        if (leftCount >= rightCount && leftCount >= noneCount && leftCount >= centerCount) {
            location = PropLocation.LEFT;
        } else if (rightCount >= leftCount && rightCount >= noneCount && rightCount >= centerCount) {
            location = PropLocation.RIGHT;
        } else if (centerCount >= noneCount){
            location = PropLocation.CENTER;
        } else {
            location = PropLocation.NONE;
        }

        telemetry.addData("PROP LOCATION: ", location);
        telemetry.update();

        Arm.rotateWorm(0);





        switch (location) {
            case LEFT:
                drive.followTrajectorySequence(toSpikeLeft);

                Auxiliaries.placePixelOnSpikeStripRight();
                sleep(spikeStripPixelPlacementSleep);
                Auxiliaries.retractPixelPlacerRight();

                drive.followTrajectorySequence(toBackdropLeft);

                Auxiliaries.placePixelOnBackdropLeft();
                sleep(backdropPixelPlacementSleep);
                Auxiliaries.retractPixelPlacerLeft();

                drive.followTrajectorySequence(toParkLeft);
                break;
            case RIGHT:
                drive.followTrajectorySequence(toSpikeRight);

                Auxiliaries.placePixelOnSpikeStripRight();
                sleep(spikeStripPixelPlacementSleep);
                Auxiliaries.retractPixelPlacerRight();

                drive.followTrajectorySequence(toBackdropRight);

                Auxiliaries.placePixelOnBackdropLeft();
                sleep(backdropPixelPlacementSleep);
                Auxiliaries.retractPixelPlacerLeft();

                drive.followTrajectorySequence(toParkRight);
                break;
            default: // Center and None
                drive.followTrajectorySequence(test1);

                drive.followTrajectorySequence(test2);

                drive.followTrajectorySequence(test3);
                break;
        }

        sleep(20000); // Stop the program from looping
    }
}
