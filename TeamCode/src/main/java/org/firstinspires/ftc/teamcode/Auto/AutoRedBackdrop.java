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

@Autonomous(name = "Auto Red - Backdrop", group = "Red")
@Config
public class AutoRedBackdrop extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;

    OpenCvCamera camera;

    MecanumDriveBase drive;

    int spikeMarkPixelPlaceSleep = 1000;
    int backdropPixelPlaceSleep  = 800;

    int leftCount   = 0;
    int centerCount = 0;
    int rightCount  = 0;
    int noneCount   = 0;

    int cameraMonitorViewId;

    TrajectorySequence toSpikeLeft, toSpikeCenter, toSpikeRight;
    TrajectorySequence toBackdropLeft, toBackdropCenter, toBackdropRight;
    TrajectorySequence toParkLeft, toParkCenter, toParkRight;

    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetector(PropColor.RED);
        drive = new MecanumDriveBase(hardwareMap);

        drive.init();

        Arm.init(hardwareMap);
        Auxiliaries.init(hardwareMap);

        Arm.update(false);

        Pose2d startPose = new Pose2d(11, -63, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        toSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, -34))
                .lineTo(new Vector2d(11, -11))
                .strafeTo(new Vector2d(6, -11))
                .build();

        toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(49, -12))
                .strafeTo(new Vector2d(49, -35))
                .lineTo(new Vector2d(51, -35))
                .build();

        toParkLeft = drive.trajectorySequenceBuilder(toBackdropLeft.end())
                .lineTo(new Vector2d(45, -35))
                .strafeTo(new Vector2d(45, -64))
                .build();

        toSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, -60))
                .strafeTo(new Vector2d(28, -60))
                .lineTo(new Vector2d(28, -28))
                .strafeTo(new Vector2d(14, -28))
                .lineTo(new Vector2d(14, -12))
                .strafeTo(new Vector2d(18, -12))
                .build();

        toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(45, -12))
                .strafeTo(new Vector2d(45, -42))
                .lineTo(new Vector2d(51, -42))
                .build();

        toParkCenter = drive.trajectorySequenceBuilder(toBackdropCenter.end())
                .lineTo(new Vector2d(45, -42))
                .strafeTo(new Vector2d(45, -63))
                .build();

        toSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, -60))
                .strafeTo(new Vector2d(33, -60))
                .lineTo(new Vector2d(33, -35))
                .strafeTo(new Vector2d(28, -35))
                .strafeTo(new Vector2d(33, -35))
                .lineTo(new Vector2d(33, -12))
                .strafeTo(new Vector2d(29, -12))
                .build();

        toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(45, -12))
                .strafeTo(new Vector2d(45, -49))
                .lineTo(new Vector2d(51, -49))
                .build();

        toParkRight = drive.trajectorySequenceBuilder(toBackdropRight.end())
                .lineTo(new Vector2d(45, -50))
                .strafeTo(new Vector2d(45, -63))
                .build();

        cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(propDetector);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Failed to open camera due to error code", errorCode);
                telemetry.update();
            }
        });

        Arm.rotateWorm(1400);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        for (int i = 0; i <= 20; i++) {
            switch (propDetector.getPropLocation()) {
                case LEFT:
                    leftCount += 1;
                    break;
                case RIGHT:
                    rightCount += 1;
                    break;
                case CENTER:
                    centerCount += 1;
                    break;
                case NONE:
                    noneCount += 1;
                    break;
            }
        }

        if (leftCount >= rightCount && leftCount >= noneCount && leftCount >= centerCount) {
            location = PropLocation.LEFT;
        } else if (rightCount >= leftCount && rightCount >= noneCount && rightCount >= centerCount) {
            location = PropLocation.RIGHT;
        } else if (centerCount >= noneCount) {
            location = PropLocation.CENTER;
        } else {
            location = PropLocation.NONE;
        }

        Arm.rotateWorm(0);

        telemetry.addData("PROP LOCATION ", location);
        telemetry.update();

        switch (location) {
            case LEFT:
                drive.followTrajectorySequence(toSpikeLeft);

                Auxiliaries.placePixelOnSpikeStripRight();
                sleep(spikeMarkPixelPlaceSleep);
                Auxiliaries.retractPixelPlacerRight();

                drive.followTrajectorySequence(toBackdropLeft);

                Auxiliaries.placePixelOnBackdropLeft();
                sleep(backdropPixelPlaceSleep);
                Auxiliaries.retractPixelPlacerLeft();

                drive.followTrajectorySequence(toParkLeft);
                break;
            case RIGHT:
                drive.followTrajectorySequence(toSpikeRight);

                Auxiliaries.placePixelOnSpikeStripRight();
                sleep(spikeMarkPixelPlaceSleep);
                Auxiliaries.retractPixelPlacerRight();

                drive.followTrajectorySequence(toBackdropRight);

                Auxiliaries.placePixelOnBackdropLeft();
                sleep(backdropPixelPlaceSleep);
                Auxiliaries.retractPixelPlacerLeft();

                drive.followTrajectorySequence(toParkRight);
                break;
            default: // Center or None
                drive.followTrajectorySequence(toSpikeCenter);

                Auxiliaries.placePixelOnSpikeStripRight();
                sleep(spikeMarkPixelPlaceSleep);
                Auxiliaries.retractPixelPlacerRight();

                drive.followTrajectorySequence(toBackdropCenter);

                Auxiliaries.placePixelOnBackdropLeft();
                sleep(backdropPixelPlaceSleep);
                Auxiliaries.retractPixelPlacerLeft();

                drive.followTrajectorySequence(toParkCenter);
                break;
        }

        sleep(20000); // This prevents the program from looping
    }
}
