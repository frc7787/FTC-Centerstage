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
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.Auxiliaries;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Red - Backdrop", group = "Red")
@Config
public class AutoRedBackdrop extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;

    public static OpenCvCamera camera;

    MecanumDriveBase drive;



    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetector(PropColor.RED);
        drive        = new MecanumDriveBase(hardwareMap);

        drive.init();

        Arm.init(hardwareMap);

        Pose2d startPose = new Pose2d(11, -63, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, -34))
                .lineTo(new Vector2d(11, -11))
                .strafeTo(new Vector2d(6, -11))
                .build();

        TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, -60))
                .strafeTo(new Vector2d(28, -60))
                .lineTo(new Vector2d(28, -28))
                .strafeTo(new Vector2d(14, -28))
                .lineTo(new Vector2d(14, -12))
                .strafeTo(new Vector2d(18, -12))
                .build();

        TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11, -60))
                .strafeTo(new Vector2d(33, -60))
                .lineTo(new Vector2d(33, -35))
                .strafeTo(new Vector2d(28, -35))
                .strafeTo(new Vector2d(33, -35))
                .lineTo(new Vector2d(33, -12))
                .strafeTo(new Vector2d(29, -12))
                .build();

        TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(49, -12))
                .strafeTo(new Vector2d(49, -34))
                .lineTo(new Vector2d(52.5, -34))
                .build();

        TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(49, -12))
                .strafeTo(new Vector2d(49, -42))
                .lineTo(new Vector2d(51, -42))
                .build();

        TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(49, -12))
                .strafeTo(new Vector2d(49, -49))
                .lineTo(new Vector2d(53.5, -50))
                .build();

        TrajectorySequence toParkLeft = drive.trajectorySequenceBuilder(toBackdropLeft.end())
                .lineTo(new Vector2d(49, -34))
                .strafeTo(new Vector2d(49, -64))
                .build();

        TrajectorySequence toParkCenter = drive.trajectorySequenceBuilder(toBackdropCenter.end())
                .lineTo(new Vector2d(49, -42))
                .strafeTo(new Vector2d(49, -63))
                .build();

        TrajectorySequence toParkRight = drive.trajectorySequenceBuilder(toBackdropRight.end())
                .lineTo(new Vector2d(49, -50))
                .strafeTo(new Vector2d(49, -63))
                .build();

        int cameraMonitorViewId = hardwareMap
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

        Arm.init(hardwareMap);
        Auxiliaries.init(hardwareMap);

        Arm.update(false);

        Arm.rotateWorm(1400);

        waitForStart();

        if (isStopRequested()) { return; }

        location = propDetector.getPropLocation();

        while (opModeIsActive()) {
            int leftCount   = 0;
            int rightCount  = 0;
            int centerCount = 0;
            int noneCount   = 0;

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

            Arm.rotateWorm(25);

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

            switch (location) {
                case LEFT:
                    drive.followTrajectorySequence(toSpikeLeft);

                    Auxiliaries.placePixelOnSpikeStripRight();
                    sleep(1000);
                    Auxiliaries.retractPixelPlacerRight();

                    drive.followTrajectorySequence(toBackdropLeft);

                    Auxiliaries.placePixelOnBackdropLeft();
                    sleep(800);
                    Auxiliaries.retractPixelPlacerLeft();

                    drive.followTrajectorySequence(toParkLeft);
                    break;
                case CENTER:
                    drive.followTrajectorySequence(toSpikeCenter);

                    Auxiliaries.placePixelOnSpikeStripRight();
                    sleep(1000);
                    Auxiliaries.retractPixelPlacerRight();

                    drive.followTrajectorySequence(toBackdropCenter);

                    Auxiliaries.placePixelOnBackdropLeft();
                    sleep(800);
                    Auxiliaries.retractPixelPlacerLeft();

                    drive.followTrajectorySequence(toParkCenter);
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(toSpikeRight);

                    Auxiliaries.placePixelOnSpikeStripRight();
                    sleep(1000);
                    Auxiliaries.retractPixelPlacerRight();

                    drive.followTrajectorySequence(toBackdropRight);

                    Auxiliaries.placePixelOnBackdropLeft();
                    sleep(800);
                    Auxiliaries.retractPixelPlacerLeft();

                    drive.followTrajectorySequence(toParkRight);
                    break;
                case NONE:
                    drive.followTrajectorySequence(toSpikeCenter);

                    Auxiliaries.placePixelOnSpikeStripRight();
                    sleep(1000);
                    Auxiliaries.retractPixelPlacerRight();

                    drive.followTrajectorySequence(toBackdropCenter);

                    Auxiliaries.placePixelOnBackdropLeft();
                    sleep(800);
                    Auxiliaries.retractPixelPlacerLeft();

                    drive.followTrajectorySequence(toSpikeCenter);
                    break;
            }

            sleep(20000);
        }
    }
}
