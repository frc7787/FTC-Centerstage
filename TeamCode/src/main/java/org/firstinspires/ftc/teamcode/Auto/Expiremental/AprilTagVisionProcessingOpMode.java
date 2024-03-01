package org.firstinspires.ftc.teamcode.Auto.Expiremental;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;

import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="April Tag Vision Processing OpMode", group = "Utility")
public class AprilTagVisionProcessingOpMode extends OpMode {
    private final FirstVisionProcessor processor = new FirstVisionProcessor();
    private AprilTagProcessor aprilTagProcessor;

    private AprilTagDetection desiredTag    = null;
    private static final int DESIRED_TAG_ID = -1; // -1 locks on to any tag
    private boolean targetFound             = false;    // Set to true when an AprilTag target is detected

    private VisionPortal visionPortal;

    private FirstVisionProcessor.ParkingPosition elementPosition = FirstVisionProcessor.ParkingPosition.CENTER;

    private int minExposure;
    private int maxExposure;
    private int myExposure;
    private int minGain;
    private int maxGain;
    private int myGain;
    private int minWhiteBalance;
    private int maxWhiteBalance;
    private int myWhiteBalance;

    @Override public void init() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(660.750, 660.75, 323.034, 230.681) // C615 measured kk Dec 5 2023
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 10);

        getDefaultCameraSettings();

        myExposure     = Math.min(5, minExposure + 1);
        myGain         = maxGain;
        myWhiteBalance = (minWhiteBalance + maxWhiteBalance) / 2;

        setCameraProperties(myExposure, myGain, myWhiteBalance);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");

        elementPosition = processor.getPosition();

        telemetry.addData("POSITION: ", elementPosition);
        telemetry.update();
    } // end of init

    @Override public void loop() {
        elementPosition = processor.getPosition();

        telemetry.addData("POSITION: ", elementPosition);
        telemetry.addData("Exposure", myExposure);
        telemetry.addData("Gain", myGain);
        telemetry.addData("White Balance", myWhiteBalance);

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        int numberOfTags = currentDetections.size();

        if (numberOfTags > 0 ) {
            telemetry.addData("Number of Tags Detected", currentDetections.size());
            targetFound = true;
        } else {
            telemetry.addLine("No tags currently detected");
            targetFound = false;
        }

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    telemetry.addData("Desired Tag Detected. Id", detection.id);

                    targetFound = true;
                    desiredTag  = detection;
                    break;  // don't look any further.
                }

                targetFound = false;
                telemetry.addData("Tage Detected. Id", detection.id);
            } else {

                telemetry.addData("Size info not available Id: ", detection.id);
            }
        }

        if (gamepad1.left_bumper)        myExposure += 1;
        if (gamepad1.left_trigger > 0.5) myExposure -= 1;

        if (gamepad1.right_bumper)        myGain += 1;
        if (gamepad1.right_trigger > 0.5) myGain -= 1;

        if (gamepad1.dpad_up)   myWhiteBalance += 1;
        if (gamepad1.dpad_down) myWhiteBalance -= 1;

        setCameraProperties(
                Range.clip(myExposure, minExposure, maxExposure),
                Range.clip(myGain, minGain, maxGain),
                Range.clip(myWhiteBalance, minWhiteBalance, maxWhiteBalance));

        telemetry.update();
    }

    private void getDefaultCameraSettings() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            throw new NullPointerException();
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();

            while (true) { // Wait for camera to open
                if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                    break;
                }
            }

            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        minWhiteBalance = whiteBalanceControl.getMinWhiteBalanceTemperature();
        maxWhiteBalance = whiteBalanceControl.getMaxWhiteBalanceTemperature();
        myWhiteBalance  = whiteBalanceControl.getWhiteBalanceTemperature();
    }

    private void setCameraProperties(int exposureMS, int gain, int white) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            throw new NullPointerException();
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();

            while (true) {
                if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                    break;
                }
            }

            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(white);
    }
}