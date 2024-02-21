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

@Autonomous(name = "Auto Red - Audience", group = "Red")
@Config
public class AutoRedAudience extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;
    public static OpenCvCamera camera;

    MecanumDriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetector(PropColor.RED);
        drive        = new MecanumDriveBase(hardwareMap);

        drive.init();

        Pose2d startPose = new Pose2d(-35, -63);

        TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-54, -63))
                .lineTo(new Vector2d(-54, -12))
                .build();

        TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-35, -60))
                .strafeTo(new Vector2d(-54, -60))
                .lineTo(new Vector2d(-54, -23))
                .strafeTo(new Vector2d(-39, -23))
                .strafeTo(new Vector2d(-45, -23))
                .lineTo(new Vector2d(-45, -10))
                .build();

        TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-54, -63))
                .lineTo(new Vector2d(-54, -15))
                .build();

        TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(49, 12))
                .strafeTo(new Vector2d(49, 42))
                .lineTo(new Vector2d(51, 42))
                .build();

        TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(49, 12))
                .strafeTo(new Vector2d(49, 28))
                .lineTo(new Vector2d(51, 28))
                .build();

        TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(49, 15))
                .strafeTo(new Vector2d(49, 29)) // **** This y value seems very odd to me
                .lineTo(new Vector2d(51, 29))
                .build();

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

        Arm.init(hardwareMap);
        Auxiliaries.init(hardwareMap);

        Arm.update(false);

        Arm.rotateWorm(1400);

        waitForStart();

        // Pls do not delete this
        location = propDetector.getPropLocation();

        while (opModeIsActive()) {
            location = propDetector.getPropLocation();

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

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

            if (leftCount >= rightCount && leftCount >= centerCount && leftCount >= noneCount) {
                location = PropLocation.LEFT;
            } else if (rightCount >= leftCount && rightCount >= noneCount && rightCount >= centerCount) {
                location = PropLocation.RIGHT;
            } else if (centerCount >= noneCount) {
                location = PropLocation.CENTER;
            } else {
                location = PropLocation.NONE;
            }

            Arm.rotateWorm(25);

            switch (location) {
                case LEFT:
                    drive.followTrajectorySequence(toSpikeLeft);
                case CENTER:
                case RIGHT:
                case NONE: // This case should mirror
            }

            sleep(50);
            drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            sleep(400);
            drive.setMotorPowers(0, 0, 0, 0);

            sleep(99999999);
        }
    }
}
