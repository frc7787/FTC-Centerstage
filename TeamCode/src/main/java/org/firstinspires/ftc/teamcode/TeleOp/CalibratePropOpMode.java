package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Auto.Utility.CalibratePropDetector;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.LEFT;
import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.NONE;
import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.RIGHT;


// Uses the non-Easy VisionPortal initialization,
// Works with FTC Dashboard, which displays the colour elements through FirstVisionProcessor
// Incorporates AprilTagProcessor, which show up on external HDMI but not FTC Dashboard
// https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_init/visionportal-init.html
// https://github.com/acmerobotics/ftc-dashboard/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/VisionPortalStreamingOpMode.java
// upload webcam configuration file to the REV Control Hub!
// https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/managing_control_hub/Managing-a-Control-Hub.html#uploading-a-custom-webcam-calibration-file-instructions
// https://www.reddit.com/r/LogitechG/comments/lmlfal/how_to_add_un_supported_webcam_to_logitech_capture/
// webcam lens intrinsics tutorial: https://youtu.be/bTcCY3DZM0k?si=EerbST3UnBUprVwY

@TeleOp(name="CalibratePropOpMode", group = "Utility")
public class CalibratePropOpMode extends OpMode {
    final CalibratePropDetector processor = new CalibratePropDetector();  // element detection pipeline
    private AprilTagProcessor aprilTagProcessor;  // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected

    PropLocation elementPosition = NONE;


    //private FirstVisionProcessor visionProcessor;

    private VisionPortal visionPortal;

    //private ParkingPosition elementPosition = ParkingPosition.CENTER;

    private int     myExposure  ;
    private int     minExposure ;
    private int     maxExposure ;
    private int     myGain      ;
    private int     minGain ;
    private int     maxGain ;
    private int     minWhiteBalance;
    private int     maxWhiteBalance;
    private int     myWhiteBalance;

    boolean thisExpUp = false;
    boolean thisExpDn = false;
    boolean thisGainUp = false;
    boolean thisGainDn = false;
    boolean thisWhiteBalanceUp = false;
    boolean thisWhiteBalanceDn = false;

    boolean lastExpUp = false;
    boolean lastExpDn = false;
    boolean lastGainUp = false;
    boolean lastGainDn = false;
    boolean lastWhiteBalanceUp = false;
    boolean lastWhiteBalanceDn = false;


    @Override
    public void init() {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                //.setTagLibrary(myAprilTagLibrary)
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
                .setCameraResolution(new Size(320, 240))
                //.setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();



        //visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor, processor);

        FtcDashboard.getInstance().startCameraStream(processor, 0);

        /*
        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);

        // myExposureControl = vuforia.getCamera().getControl(ExposureControl.class);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //FtcDashboard.getInstance().startCameraStream(visionProcessor, 10);

         */

        getCameraSetting();
        myExposure = Math.min(5, minExposure+1);
        myGain = maxGain;
        myWhiteBalance = (minWhiteBalance + maxWhiteBalance) / 2;
        setManualExposure(myExposure, myGain, myWhiteBalance);

        // Wait for the match to begin.
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");

        elementPosition=processor.getPosition();
        telemetry.addData("POSITION: ", elementPosition);
        telemetry.update();

    } // end of init

    @Override
    public void init_loop() {

    } // end of init_loop

    @Override
    public void start() {

    } // end of start
    @Override
    public void loop() {
        elementPosition = processor.getPosition();
        telemetry.addData("POSITION: ", elementPosition);

        telemetry.addData("Exposure (left button/trigger) :","%d  (%d - %d)", myExposure, minExposure, maxExposure);
        telemetry.addData("Gain (right button/trigger):","%d  (%d - %d)", myGain, minGain, maxGain);
        telemetry.addData("White Balance: (dpad up/down)","%d (%d - %d)", myWhiteBalance, minWhiteBalance, maxWhiteBalance);

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        int numTags = currentDetections.size();
        if (numTags > 0 )
            telemetry.addData("Tag", "####### %d Detected  ######", currentDetections.size());
        else {
            telemetry.addData("Tag", "----------- none - ----------");
            targetFound = false;
        }
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        // check to see if we need to change exposure or gain.
        thisExpUp = gamepad1.left_bumper;
        thisExpDn = gamepad1.left_trigger > 0.25;
        thisGainUp = gamepad1.right_bumper;
        thisGainDn = gamepad1.right_trigger > 0.25;
        thisWhiteBalanceUp = gamepad1.dpad_up;
        thisWhiteBalanceDn = gamepad1.dpad_down;

        // look for clicks to change exposure
        if (thisExpUp && !lastExpUp) {
            myExposure = Range.clip(myExposure + 1, minExposure, maxExposure);
            setManualExposure(myExposure, myGain, myWhiteBalance);
        } else if (thisExpDn && !lastExpDn) {
            myExposure = Range.clip(myExposure - 1, minExposure, maxExposure);
            setManualExposure(myExposure, myGain, myExposure);
        }

        // look for clicks to change the gain
        if (thisGainUp && !lastGainUp) {
            myGain = Range.clip(myGain + 1, minGain, maxGain );
            setManualExposure(myExposure, myGain, myWhiteBalance);
        } else if (thisGainDn && !lastGainDn) {
            myGain = Range.clip(myGain - 1, minGain, maxGain );
            setManualExposure(myExposure, myGain, myWhiteBalance);
        }

        // look for clicks to change the white balance
        if (thisWhiteBalanceUp && !lastWhiteBalanceUp) {
            myWhiteBalance = Range.clip(myWhiteBalance + 100, minWhiteBalance, maxWhiteBalance );
            setManualExposure(myExposure, myGain, myWhiteBalance);
        } else if (thisWhiteBalanceDn && !lastWhiteBalanceDn) {
            myWhiteBalance = Range.clip(myWhiteBalance - 100, minWhiteBalance, maxWhiteBalance );
            setManualExposure(myExposure, myGain, myWhiteBalance);
        }


        lastExpUp = thisExpUp;
        lastExpDn = thisExpDn;
        lastGainUp = thisGainUp;
        lastGainDn = thisGainDn;
        lastWhiteBalanceUp = thisWhiteBalanceUp;
        lastWhiteBalanceDn = thisWhiteBalanceDn;
        telemetry.update();

    } // end of loop

    /*
    Read this camera's minimum and maximum Exposure and Gain settings.
    Can only be called AFTER calling initAprilTag();
 */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {

            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/webcam_controls/eval/eval.html
        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/webcam_controls/index.html
        // https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_webcams/visionportal-webcams.html#arducam-global-shutter-120-fps

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        Boolean gotWhiteBalanceControl = whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        minWhiteBalance = whiteBalanceControl.getMinWhiteBalanceTemperature();
        maxWhiteBalance = whiteBalanceControl.getMaxWhiteBalanceTemperature();
        myWhiteBalance = (int)whiteBalanceControl.getWhiteBalanceTemperature();

    } // end of getCameraSettings

    private boolean    setManualExposure(int exposureMS, int gain, int white) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.

        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        // Set White Balance.
        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.MANUAL) {
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        }
        whiteBalanceControl.setWhiteBalanceTemperature(myWhiteBalance);

        return (true);

    } // end of boolean setManualExposure


} // end of public class
