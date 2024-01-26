package org.firstinspires.ftc.teamcode.Auto;

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

@Autonomous(name = "Auto Red - Backdrop")
public class AutoRedBackdrop extends LinearOpMode {
    PropDetector propDetector;
    PropLocation location;
    public static OpenCvCamera camera;

    MecanumDriveBase drive;

    Intake intake;


    @Override
    public void runOpMode() throws InterruptedException {
        Rect cropRectangle = new Rect(130, 120, 190, 120);

        propDetector = new PropDetector(PropColor.RED, cropRectangle);
        drive  = new MecanumDriveBase(hardwareMap);
        intake = new Intake(hardwareMap);

        drive.init();

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId
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
            switch (location) {
                case LEFT:
                    // THIS IS ACTUALLY CENTER LINE

                    // Drive forward until you hit the line
                    drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
                    sleep(2500);

                    // Outtake the pixel
                    intake.outtake(0.4);
                    sleep(1000);

                    break;
                case NONE:
                    // THIS IS ACTUALLY LEFT LINE

                    // Drive off the wall
                    drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
                    sleep(500);
                    // Turn slightly and drive forward to the line
                    drive.turn(3.14 * 0.10);
                    drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
                    sleep(1500);
                    // Outtake the pixel
                    intake.outtake(0.4);
                    sleep(1000);

                    break;
                case RIGHT:
                    // ACTUALLY RIGHT LINE

                    // Drive off the wall
                    drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
                    sleep(500);
                    // Turn to the right and drive to the line
                    drive.turn(-3.14 * 0.10);
                    drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
                    sleep(1500);
                    // Outtake the pixel
                    intake.outtake(0.4);
                    sleep(1000);

                    break;
            }

            sleep(99999999);
        }
    }
}
