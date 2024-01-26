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

@Autonomous(name = "Auto Blue - Audience", group = "Blue")
@Config
public class AutoBlueAudience extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;
    public static OpenCvCamera camera;

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
        Rect cropRectangle = new Rect(130, 120, 190, 120);

        propDetector = new PropDetector(PropColor.BLUE, cropRectangle);
        drive        = new MecanumDriveBase(hardwareMap);
        intake       = new Intake(hardwareMap);

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
        location = propDetector.getLocation();

        while (opModeIsActive()) {
            location = propDetector.getLocation();

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

            sleep(5000);

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
            }

            sleep(99999999);
        }
    }
}
