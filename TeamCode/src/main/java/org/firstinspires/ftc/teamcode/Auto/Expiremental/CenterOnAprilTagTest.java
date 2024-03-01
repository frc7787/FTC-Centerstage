package org.firstinspires.ftc.teamcode.Auto.Expiremental;

1 of 1
        apriltag auto drive
        Inbox

        Kas Karim
        Attachments
        2:55 PM (1 hour ago)
        to me


        One attachment
        •  Scanned by Gmail
        /* Copyright (c) 2017 FIRST. All rights reserved.
         *
         * Redistribution and use in source and binary forms, with or without modification,
         * are permitted (subject to the limitations in the disclaimer below) provided that
         * the following conditions are met:
         *
         * Redistributions of source code must retain the above copyright notice, this list
         * of conditions and the following disclaimer.
         *
         * Redistributions in binary form must reproduce the above copyright notice, this
         * list of conditions and the following disclaimer in the documentation and/or
         * other materials provided with the distribution.
         *
         * Neither the name of FIRST nor the names of its contributors may be used to endorse or
         * promote products derived from this software without specific prior written permission.
         *
         * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
         * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
         * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
         * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
         * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
         * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
         * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
         * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
         * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
         * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
         * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
         */

        package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@Autonomous (name="TestAutoColourRoadrunnerAprilDec10", group = "Linear OpMode")
public class CenterOnAprilTagTest extends LinearOpMode {

    // Declare OpMode members.

    final FirstVisionProcessor processor = new FirstVisionProcessor();  // element detection pipeline
    private AprilTagProcessor aprilTagProcessor;  // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private static final int DESIRED_TAG_ID = 3;     // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected

    private VisionPortal visionPortal;

    private int myExposure     = 3;
    private int myGain         = 255;
    private int myWhiteBalance = 4800;

    public FirstVisionProcessor.ParkingPosition elementPosition = FirstVisionProcessor.ParkingPosition.CENTER;

    final double DESIRED_DISTANCE = 4.0;

    // CHANGED test values
    final double SPEED_GAIN  =  0.015;
    final double STRAFE_GAIN =  0.01;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.001;  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.25;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.15;   //  Clip the turn speed to this max value (adjust for your robot)

    @Override
    public void runOpMode() {
        DriveBase.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initVideo();
        // Wait for the match to begin.
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");

        elementPosition = processor.getPosition();
        telemetry.addData("POSITION: ", elementPosition);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Setting Trajectories", "Ready to begin");
            telemetry.update();

            telemetry.addData("Follow Sequence 1", "On my way");

            telemetry.addData("End of Sequence 2", "Switching to April Tag Drive");
            sleep(1000);
            aprilTagBackdrop();

            telemetry.update();
            sleep(30000);
        } // end of while opModeIsActive
    } // end of runOpMode

    public void initVideo() {

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

        FtcDashboard.getInstance().startCameraStream(processor, 0);
        setManualExposure(myExposure, myGain, myWhiteBalance);
    }

    private void setManualExposure(int exposureMS, int gain, int white) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            throw new NullPointerException();
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Set Manual Exposure, Camera", "Waiting");
            telemetry.update();

            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }

            telemetry.addData("Set Manual Exposure, Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);

            // Set White Balance.
            WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
            whiteBalanceControl.setWhiteBalanceTemperature(myWhiteBalance);

        }
    } // end of boolean setManualExposure

    public void aprilTagBackdrop() {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        boolean targetReached   = false;    // Set to true when the AprilTage target has been reached
        double  drive   = 0;
        double  strafe  = 0;
        double  turn    = 0;

        desiredTag  = null;

        telemetry.addData("Inside April Tag Drive", "Embark on adventure");
        sleep(1000);

        while (opModeIsActive() && !targetReached) {
            targetFound = false;

            // Step through the list of detected tags and look for a matching tag
            telemetry.addData("Detecting April Tags", "Now");
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        telemetry.addData("April Tag found", detection.id);
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
            } // end for detection

            if (targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = -1.0*Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                turn   = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                telemetry.addData("Distance", "Range %5.2f, RangeError %5.2f, HeadingError %5.2f, TurnError %5.2f ", desiredTag.ftcPose.range ,rangeError, headingError, yawError);

                // Apply desired axes motions to the drivetrain.
                if (rangeError <= 0.1) {
                    targetReached = true;
                    telemetry.addData("Target Reached!! rangeError:", rangeError);
                } else {
                    DriveBase.driveManualRobotCentric(drive, strafe, turn);
                    sleep(10);
                }
                telemetry.update();
            } // end of if targetFound

        } // end of while !targetReached

        moveRobot(drive, strafe, turn);

    } // end of aprilTagBackdrop

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        telemetry.addData("Move Robot", "Giddyup!");
        drive.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
        drive.update();
    } // end of moveRobot

} // end of public class