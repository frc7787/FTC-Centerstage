package org.firstinspires.ftc.teamcode.Auto.Utility;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Auto.Utility.PropColor;

import java.util.ArrayList;

public abstract class AutoMode extends LinearOpMode {

    public enum AutoPath {
        SHORT,
        LONG
    }

    public enum AutoLocation {
        LEFT,
        RIGHT
    }

    public AutoPath  autoPath;
    public PropColor autoColor;
    public AutoLocation autoLocation;

    private boolean propDetectorColorIsSet = false;
    private boolean isInit = false;

    public WebcamName webcam1, webcam2;

    public int pixelStackTagId, backDropTagId;

    public OpenCvCamera camera;

    public AprilTagProcessor aprilTag;

    public AprilTagDetection pixelStackTag, backDropTag;

    public VisionPortal visionPortal;

    public PropDetector propDetector;

    public final MecanumDriveBase driveBase = new MecanumDriveBase(hardwareMap);
    public final Arm arm                    = new Arm(hardwareMap);

    StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(
            hardwareMap,
            new ArrayList<>(),
            new ArrayList<>()
    );

    public void autoInit(@NonNull PropColor color, @NonNull AutoPath path, @NonNull AutoLocation location) {
        if (isInit) { return; }

        autoColor    = color;
        autoPath     = path;
        autoLocation = location;

        initPropDetector(autoColor);
        initCamera();
        initAprilTag();

        isInit = true;
    }

    public final void initPropDetector(@NonNull PropColor color) {
        if (!propDetectorColorIsSet) {
            propDetector = new PropDetector(color);
            propDetectorColorIsSet = true;
        }
    }

    @NonNull public final TrajectorySequence getSpikeStripTrajectory() {

        TrajectorySequence spikeStripTrajectory = null;

        switch (autoColor) {
            case BLUE:
                switch (autoLocation) {
                    case LEFT:
                        spikeStripTrajectory = driveBase.trajectorySequenceBuilder(new Pose2d(-36, 67, Math.toRadians(270)))
                                .lineTo(new Vector2d(-36.32, 34.45))
                                .build();
                        break;
                    case RIGHT:
                        spikeStripTrajectory = driveBase.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(270.00)))
                                .lineTo(new Vector2d(12.49, 34.30))
                                .build();
                        break;
                }
                break;
            case RED:
                switch (autoLocation) {
                    case LEFT:
                        spikeStripTrajectory = driveBase.trajectorySequenceBuilder(new Pose2d(-36.61, -67.38, Math.toRadians(90.00)))
                                .lineTo(new Vector2d(-36.32, -34.01))
                                .build();
                        break;
                    case RIGHT:
                        spikeStripTrajectory = driveBase.trajectorySequenceBuilder(new Pose2d(12.49, -65.36, Math.toRadians(90.00)))
                            .lineTo(new Vector2d(12.35, -34.16))
                            .build();
                        break;
                }
                break;
        }
        return spikeStripTrajectory;
    }

    public void getPixelFromStackAndPlaceOnBackdrop(int iterations) {
        if (!isInit) {
            throw new RuntimeException("Must initialize auto before performing any function");
        }

        int iteration = 0;

        while (iteration <= iterations) {

            visionPortal.setActiveCamera(webcam1);

            detectAprilTag(pixelStackTagId, pixelStackTag);

            centerOnAprilTag(pixelStackTag, 5);

            // Pick up the pixel

            visionPortal.setActiveCamera(webcam2); // Switch to the camera on the back of the robot

            // We need to build a new trajectory sequence with the current position of the robot
            // If we do it at the top, we will get a stale value for the current robot Pose
            driveBase.followTrajectorySequence(toBackdropTrajectoryBuilder());

            detectAprilTag(pixelStackTagId, backDropTag);

            centerOnAprilTag(pixelStackTag, 5);

            // We need to build a new trajectory sequence with the current position of the robot
            // If we do it at the top, we will get a stale value for the current robot Pose
            driveBase.followTrajectorySequence(fromBackDropTrajectoryBuilder());

            // Drop of pixel with intake that does not currently exist

            iteration += 1;
        }
    }


    @NonNull public TrajectorySequence toBackdropTrajectoryBuilder() {
        localizer.update();

        TrajectorySequence outputTrajectorySequence = null;

        switch (autoColor) {
            case RED:
                switch (autoPath) {
                    case LONG:
                        outputTrajectorySequence = driveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                                .lineTo(new Vector2d(37.05, -10.90))
                                .addTemporalMarker(0.5, () -> arm.moveToPosition(0,0)) // TODO Figure out position
                                .lineTo(new Vector2d(-36.61, -11.05))
                                .lineTo(new Vector2d(-54.38, -35.03))
                                .build();
                        break;
                    case SHORT:
                        outputTrajectorySequence = driveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                                .lineTo(new Vector2d(39.07, -36.32))
                                .addTemporalMarker(0.5, () -> arm.moveToPosition(0,0))
                                .build();
                        break;
                }
            case BLUE:
                switch (autoPath) {
                    case LONG:
                        outputTrajectorySequence = driveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                                .lineTo(new Vector2d(-33.58, 10.76))
                                .addTemporalMarker(0.5, () -> arm.moveToPosition(0, 0)) // TODO Figure out position
                                .lineTo(new Vector2d(37.91, 10.90))
                                .lineTo(new Vector2d(37.91, 38.35))
                                .build();
                        break;
                    case SHORT:
                        outputTrajectorySequence = driveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                                .lineTo(new Vector2d(39.07, 36.32))
                                .addTemporalMarker(0.5, () -> arm.moveToPosition(0,0))
                                .build();
                        break;
                }
        }

        return outputTrajectorySequence;
    }

    @NonNull private TrajectorySequence fromBackDropTrajectoryBuilder() {
        localizer.update();

        TrajectorySequence outputTrajectorySequence = null;

        switch (autoColor) {
            case RED:
                switch (autoPath) {
                    case LONG:
                        outputTrajectorySequence = driveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                                .lineTo(new Vector2d(36.90, -11.63))
                                .lineTo(new Vector2d(-38.06, -11.63))
                                .lineTo(new Vector2d(-56.26, -35.75))
                                .build();
                        break;
                    case SHORT:
                        outputTrajectorySequence = driveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                                .lineTo(new Vector2d(-38.78, -36.18))
                                .build();
                }
            case BLUE:
                switch (autoPath) {
                    case LONG:
                        outputTrajectorySequence = driveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                                .lineTo(new Vector2d(33.58, 14.08))
                                .lineTo(new Vector2d(-36.47, 13.50))
                                .lineTo(new Vector2d(-54.81, 35.46))
                                .build();
                        break;
                    case SHORT:
                        outputTrajectorySequence = driveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                                .lineTo(new Vector2d(-38.78, 36.18))
                                .build();
                        break;
                }
        }

        return outputTrajectorySequence;

    }

    public final void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTag)
                .build();
    }

    public final void initCamera() {
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
    }

    public final void centerOnAprilTag(@NonNull AprilTagDetection tag, int distance) {
        // Epic code that I am going to make Kai write cause I don't feel like figuring it out
    }

    public final AprilTagDetection detectAprilTag(int id, @NonNull AprilTagDetection output) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.metadata != null) {
                if ((backDropTagId < 0) || (detection.id == id)) {
                    output = detection;
                }
            }
        }
        return output;
    }

    abstract public void runOpMode();

    public void waitForAutoStart() {
        if (!isInit) {
            throw new RuntimeException("Must call 'autoInit' before starting the robot");
        }

        waitForStart();
    }
}
