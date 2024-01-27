package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropColor;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorManual;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Blue Manual - Audience", group = "Blue")
@Config
public class AutoBlueAudienceManual extends LinearOpMode {
    Rect cropRectangle = new Rect(0, 120, 320, 120);

    PropDetectorManual propDetector;
    //PropDetector propDetector;
    private VisionPortal visionPortal;
    public static int     myExposure  = 3;
    public static int     myGain = 255     ;
    public static int     myWhiteBalance = 2000;

    PropLocation location;
    //public static OpenCvCamera camera;

    MecanumDriveBase drive;

    Intake intake;

    public static int CENTER_FORWARD_SLEEP = 1170;
    public static int LEFT_FORWARD_SLEEP   = 500;
    public static int RIGHT_FORWARD_SLEEP  = 500;
    public static int LEFT_TURN_SLEEP      = 600;
    public static int RIGHT_TURN_SLEEP     = 450;

    public static double RIGHT_ANGLE = -0.514;
    public static double LEFT_ANGLE  = 0.714;

    @Override
    public void runOpMode() throws InterruptedException {

        propDetector = new PropDetectorManual(PropColor.BLUE, cropRectangle);
        //propDetector = new PropDetector(PropColor.BLUE, cropRectangle);
        //drive        = new MecanumDriveBase(hardwareMap);
        //intake       = new Intake(hardwareMap);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(propDetector)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240))
                //.setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(propDetector, 0);

        //drive.init();

        waitForStart();

        // Pls do not delete this
        location = propDetector.getLocation();

        while (opModeIsActive()) {
            location = propDetector.getLocation();

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

            //sleep(500);
/*
            switch (location) {
                case LEFT:
                    // THIS IS ACTUALLY CENTER LINE

                    // Drive forward until you hit the line
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(CENTER_FORWARD_SLEEP);
                    drive.setMotorPowers(0, 0, 0, 0);

                    break;
                case NONE:
                    // THIS IS ACTUALLY LEFT LINE

                    // Drive off the wall
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(LEFT_FORWARD_SLEEP);
                    // Turn slightly and drive forward to the line
                    drive.turn(LEFT_ANGLE);
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(LEFT_TURN_SLEEP);
                    drive.setMotorPowers(0, 0, 0, 0);

                    break;
                case RIGHT:
                    // ACTUALLY RIGHT LINE

                    // Drive off the wall
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(RIGHT_FORWARD_SLEEP);
                    // Turn to the right and drive to the line
                    drive.turn(RIGHT_ANGLE);
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(RIGHT_TURN_SLEEP);
                    drive.setMotorPowers(0, 0, 0, 0);

                    break;
            }*/

            sleep(500);
        }
    }

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

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(myExposure, TimeUnit.MILLISECONDS);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(myGain);


        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        Boolean gotWhiteBalanceControl = whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);

        // Set White Balance.
        if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.MANUAL) {
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        }
        whiteBalanceControl.setWhiteBalanceTemperature(myWhiteBalance);

    } // end of getCameraSettings
}