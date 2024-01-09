package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Properties.BACKDROP_ELEVATOR_POS;
import static org.firstinspires.ftc.teamcode.Properties.BACKDROP_WORM_POS;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.AprilTag;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
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

    @Override public void runOpMode() {}
//
//    public static enum AutoPath {
//        SHORT,
//        LONG
//    }
//
//    private WebcamName webcam1, webcam2;
//
//    public int pixelStackTagId = 7;
//    public int backDropStackTagId = 5;
//
//    AprilTag aprilTagDetection = new AprilTag();
//
//    OpenCvCamera camera;
//
//    AprilTagDetection pixelStackTag, backDropTag;
//
//    VisionPortal visionPortal;
//
//    StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(
//            hardwareMap,
//            new ArrayList<>(),
//            new ArrayList<>()
//    );
//
//    PropDetector propDetector = new PropDetector(PropDetector.PropColor.RED);
//
//    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
//
//    MecanumDriveBase roadRunnerDriveBase = new MecanumDriveBase(hardwareMap);
//
//    Arm arm             = new Arm(hardwareMap);
//    Intake intake       = new Intake(hardwareMap);
//
//    TrajectorySequence toSpikeStrip = roadRunnerDriveBase.trajectorySequenceBuilder(new Pose2d(-36.32, -63.77, Math.toRadians(90.00)))
//            .lineTo(new Vector2d(-36.18, -34.01))
//            .build();
//
//    @Override public void runOpMode() {
//        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
//        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
//
//        localizer = new StandardTrackingWheelLocalizer(
//                hardwareMap,
//                new ArrayList<>(),
//                new ArrayList<>()
//        );
//
//        localizer.setPoseEstimate(startPose);
//
//        arm.init();
//        intake.init();
//
//        initCamera();
//
//        PropDetector.PropLocation location;
//
//        visionPortal = aprilTagDetection.initAprilTag(webcam1, webcam2);
//
//        // -----------------------------------------------------------------------------------------
//        // AUTO START
//        // -----------------------------------------------------------------------------------------
//
//        waitForStart();
//
//        location = propDetector.getLocation();
//
//        localizer.update();
//
//        // ---------- Move To Spike Strip ---------- //
//
//        roadRunnerDriveBase.followTrajectorySequence(toSpikeStrip);
//
//        // ---------- Detect Prop, And Place Spike Pixel ---------- //
//
//        switch (location) {
//            case LEFT:
//                roadRunnerDriveBase.turn(90);
//                // Release the preloaded pixel somehow???
//                break;
//            case RIGHT:
//                roadRunnerDriveBase.turn(-90);
//                // Release the preloaded pixel somehow???
//                roadRunnerDriveBase.turn(180);
//                break;
//            case NONE:
//                // Release the preloaded pixel somehow???
//                roadRunnerDriveBase.turn(90);
//                break;
//        }
//
//        // ---------- Main Auto ---------- //
//
//        getPixelFromStackAndPlaceOnBackdrop(5);
//    }
//
//    public void getPixelFromStackAndPlaceOnBackdrop(int iterations) {
//        int iteration = 0;
//
//        while (iteration <= iterations) {
//
//            visionPortal.setActiveCamera(webcam1);
//
//            aprilTagDetection.detectAprilTag(pixelStackTagId);
//
//            aprilTagDetection.centerOnAprilTag(pixelStackTag);
//
//            intake.intake(2000); // Pick up the pixel
//
//            visionPortal.setActiveCamera(webcam2); // Switch to the camera on the back of the robot
//
//            // We need to build a new trajectory sequence with the current position of the robot
//            // If we do it at the top, we will get a stale value for the current robot Pose
//            roadRunnerDriveBase.followTrajectorySequence(toBackDropTrajectoryBuilder(PropDetector.PropColor.RED, AutoPath.LONG));
//
//            aprilTagDetection.detectAprilTag(pixelStackTagId);
//
//            aprilTagDetection.centerOnAprilTag(pixelStackTag);
//
//            // We need to build a new trajectory sequence with the current position of the robot
//            // If we do it at the top, we will get a stale value for the current robot Pose
//            roadRunnerDriveBase.followTrajectorySequence(fromBackDropTrajectoryBuilder());
//
//            // Drop of pixel with intake that does not currently exist
//
//            iteration += 1;
//        }
//    }
//
//    /**
//     * Creates a new backdrop trajectory based on the current position of the robot
//     * @return The new trajectory
//     */
//    @NonNull TrajectorySequence toBackDropTrajectoryBuilder(@NonNull PropColor color, @NonNull AutoPath path) {
//        localizer.update();
//
//        TrajectorySequence outputTrajectorySequence = null;
//
//        switch (color) {
//            case RED:
//                switch (path) {
//                    case LONG:
//                        outputTrajectorySequence = roadRunnerDriveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
//                                .lineTo(new Vector2d(37.05, -10.90))
//                                .addTemporalMarker(0.5, () -> arm.moveToPosition(0,0)) // TODO Figure out position
//                                .lineTo(new Vector2d(-36.61, -11.05))
//                                .lineTo(new Vector2d(-54.38, -35.03))
//                                .build();
//                    case SHORT:
//                        outputTrajectorySequence = roadRunnerDriveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
//                                .lineTo(new Vector2d(39.07, -36.32))
//                                .addTemporalMarker(0.5, () -> arm.moveToPosition(0,0))
//                                .build();
//                }
//            case BLUE:
//                switch (path) {
//                    case LONG:
//                        outputTrajectorySequence = roadRunnerDriveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
//                                .lineTo(new Vector2d(-33.58, 10.76))
//                                .addTemporalMarker(0.5, () -> arm.moveToPosition(0, 0)) // TODO Figure out position
//                                .lineTo(new Vector2d(37.91, 10.90))
//                                .lineTo(new Vector2d(37.91, 38.35))
//                                .build();
//                    case SHORT:
//                        outputTrajectorySequence = roadRunnerDriveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
//                                .lineTo(new Vector2d(39.07, 36.32))
//                                .addTemporalMarker(0.5, () -> arm.moveToPosition(0,0))
//                                .build();
//                }
//        }
//
//        return outputTrajectorySequence;
//    }
//
//    /**
//     * Creates a new trajectory to go from the backdrop to the pixel stack, based on the current position of the robot
//     * @return The new trajectory
//     */
//    TrajectorySequence fromBackDropTrajectoryBuilder() {
//        localizer.update();
//
//        return roadRunnerDriveBase.trajectorySequenceBuilder(localizer.getPoseEstimate())
//                .lineTo(new Vector2d(-38.49, -13.50))
//                .lineTo(new Vector2d(36.90, -13.22))
//                .addTemporalMarker(8, () -> arm.moveToPosition(BACKDROP_ELEVATOR_POS, BACKDROP_WORM_POS))
//                .lineTo(new Vector2d(36.90, -38.35))
//                .build();
//    }
//
//    void initCamera() {
//        int cameraMonitorViewId = hardwareMap
//                .appContext
//                .getResources()
//                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        camera = OpenCvCameraFactory
//                .getInstance()
//                .createWebcam(
//                        hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId
//                );
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                camera.setPipeline(propDetector);
//            }
//
//            @Override public void onError(int errorCode) {
//                telemetry.addData("Failed to open camera due to error code", errorCode);
//                telemetry.update();
//            }
//        });
//    }
}