package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropColor;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetector;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Red - Audience", group = "Red")
@Config
public class AutoRedAudience extends LinearOpMode {
    Rect cropRectangle = new Rect(0, 120, 240, 120);

    PropDetector propDetector;
    PropLocation location;
    public static OpenCvCamera camera;

    MecanumDriveBase drive;

    Intake intake;

    public static int CENTER_FORWARD_SLEEP = 1100;
    public static int LEFT_FORWARD_SLEEP   = 550;
    public static int RIGHT_FORWARD_SLEEP  = 500;
    public static int LEFT_TURN_SLEEP      = 400;
    public static int RIGHT_TURN_SLEEP     = 720;

    public static double RIGHT_ANGLE = -0.714;
    public static double LEFT_ANGLE  = 0.45;

    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetector(PropColor.RED, cropRectangle);
        drive        = new MecanumDriveBase(hardwareMap);

        drive.init();

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

        waitForStart();

        // Pls do not delete this
        location = propDetector.getPropLocation();

        while (opModeIsActive()) {
            location = propDetector.getPropLocation();

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

            int leftCount  = 0;
            int rightCount = 0;
            int noneCount  = 0;

            for (int i = 0; i <= 20;  i++) {
                switch (propDetector.getPropLocation()) {
                    case LEFT:
                        leftCount += 1;
                        break;
                    case RIGHT:
                        rightCount += 1;
                        break;
                    case NONE:
                        noneCount += 1;
                        break;
                }
            }

            if (leftCount >= rightCount && leftCount >= noneCount) {
                location = PropLocation.LEFT;
            } else if (rightCount >= leftCount && rightCount >= noneCount) {
                location = PropLocation.RIGHT;
            } else {
                location = PropLocation.NONE;
            }

            //sleep(5000);

            switch (location) {
                case LEFT:
                    // THIS IS ACTUALLY LEFT

                    // Drive off the wall
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(LEFT_FORWARD_SLEEP);
                    // Turn slightly and drive forward to the line
                    drive.turn(LEFT_ANGLE);
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(LEFT_TURN_SLEEP);
                    drive.setMotorPowers(0, 0, 0, 0);

                    break;
                case NONE:
                    // THIS IS ACTUALLY RIGHT

                    // Drive off the wall
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(RIGHT_FORWARD_SLEEP);
                    // Turn to the right and drive to the line
                    drive.turn(RIGHT_ANGLE);
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(RIGHT_TURN_SLEEP);
                    drive.setMotorPowers(0, 0, 0, 0);

                    break;
                case RIGHT:
                    // ACTUALLY CENTER
                    // Drive forward until you hit the line
                    drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
                    sleep(CENTER_FORWARD_SLEEP);
                    drive.setMotorPowers(0, 0, 0, 0);

                    break;
            }

            sleep(50);
            drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            sleep(400);
            drive.setMotorPowers(0, 0, 0, 0);

            sleep(99999999);
        }
    }
}
