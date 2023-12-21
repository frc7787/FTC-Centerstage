package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RoadRunnerDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Auto Test - Blue", group = "Test")
public class AutoModeRedLong extends LinearOpMode {

    private final int BACKDROP_ELEVATOR_POS = 500;
    private final int BACKDROP_WORM_POS     = 500;

    private WebcamName webcam1, webcam2;

    public int pixelStackTagId, backDropTagId;

    OpenCvCamera camera;

    AprilTagProcessor aprilTag;

    AprilTagDetection pixelStackTag, backDropTag;

    VisionPortal visionPortal;

    StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(
            hardwareMap,
            new ArrayList<>(),
            new ArrayList<>()
    );

    PropDetector propDetector = new PropDetector(PropDetector.PropColor.RED);

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    RoadRunnerDriveBase roadRunnerDriveBase = new RoadRunnerDriveBase(hardwareMap);

    DriveBase driveBase = new DriveBase(hardwareMap);
    Arm arm             = new Arm(hardwareMap);
    Intake intake       = new Intake(hardwareMap);

    TrajectorySequence toSpikeStrip = roadRunnerDriveBase.trajectorySequenceBuilder(new Pose2d(-36.32, -63.77, Math.toRadians(90.00)))
            .lineTo(new Vector2d(-36.18, -34.01))
            .build();

    @Override public void runOpMode() {
        localizer = new StandardTrackingWheelLocalizer(
                hardwareMap,
                new ArrayList<>(),
                new ArrayList<>()
        );

        localizer.setPoseEstimate(startPose);

        driveBase.init();
        arm.init();
        intake.init();

        initCamera();

        PropDetector.PropLocation location;

        initAprilTag();

        // -----------------------------------------------------------------------------------------
        // AUTO START
        // -----------------------------------------------------------------------------------------

        waitForStart();

        location = propDetector.getLocation();

        localizer.update();

        // ---------- Move To Spike Strip ---------- //

        roadRunnerDriveBase.followTrajectorySequence(toSpikeStrip);

        // ---------- Detect Prop, And Place Spike Pixel ---------- //

        switch (location) {
            case LEFT:
                roadRunnerDriveBase.turn(90);
                // Release the preloaded pixel somehow???
                break;
            case RIGHT:
                roadRunnerDriveBase.turn(-90);
                // Release the preloaded pixel somehow???
                roadRunnerDriveBase.turn(180);
                break;
            case NONE:
                // Release the preloaded pixel somehow???
                roadRunnerDriveBase.turn(90);
                break;
        }

        // ---------- Main Auto ---------- //

        getPixelFromStackAndPlaceOnBackdrop(5);
    }

    private void getPixelFromStackAndPlaceOnBackdrop(int iterations) {
        int iteration = 0;

        while (iteration <= iterations) {

            visionPortal.setActiveCamera(webcam1);

            detectAprilTag(pixelStackTagId, pixelStackTag);

            centerOnAprilTag(pixelStackTag, 5);

            intake.intake(2000); // Pick up the pixel

            visionPortal.setActiveCamera(webcam2); // Switch to the camera on the back of the robot

            // We need to build a new trajectory sequence with the current position of the robot
            // If we do it at the top, we will get a stale value for the current robot Pose
            roadRunnerDriveBase.followTrajectorySequence(toBackDropTrajectoryBuilder());

            detectAprilTag(pixelStackTagId, backDropTag);

            centerOnAprilTag(pixelStackTag, 5);

            // We need to build a new trajectory sequence with the current position of the robot
            // If we do it at the top, we will get a stale value for the current robot Pose
            roadRunnerDriveBase.followTrajectorySequence(fromBackDropTrajectoryBuilder());

            // Drop of pixel with intake that does not currently exist

            iteration += 1;
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Creates a new backdrop trajectory based on the current position of the robot
     * @return The new trajectory
     */
    TrajectorySequence toBackDropTrajectoryBuilder() {
        localizer.update();

        return roadRunnerDriveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                .lineTo(new Vector2d(37.05, -10.90))
                .addTemporalMarker(0.5, () -> arm.moveToPosition(0,0))
                .lineTo(new Vector2d(-36.61, -11.05))
                .lineTo(new Vector2d(-54.38, -35.03))
                .build();
    }

    /**
     * Creates a new trajectory to go from the backdrop to the pixel stack, based on the current position of the robot
     * @return The new trajectory
     */
    TrajectorySequence fromBackDropTrajectoryBuilder() {
        localizer.update();

        return roadRunnerDriveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
                .lineTo(new Vector2d(-38.49, -13.50))
                .lineTo(new Vector2d(36.90, -13.22))
                .addTemporalMarker(8, () -> arm.moveToPosition(BACKDROP_ELEVATOR_POS, BACKDROP_WORM_POS))
                .lineTo(new Vector2d(36.90, -38.35))
                .build();
    }

    void initCamera() {
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

    void centerOnAprilTag(@NonNull AprilTagDetection tag, int distance) {
    }

    AprilTagDetection detectAprilTag(int id, @NonNull AprilTagDetection output) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.metadata != null) {
                if ((backDropTagId < 0) || (detection.id == id)) {
                    output = detection;
                }
            }
        }
        return output;
    }
}