package org.firstinspires.ftc.teamcode.Auto.Production;

import static org.firstinspires.ftc.teamcode.Constants.HIGH_EXTEND_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.LOW_EXTEND_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.MED_EXTEND_POSITION;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorRed;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto RED")
public class AutoModeRed extends LinearOpMode {
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    PropDetectorRed detector = new PropDetectorRed();
    OpenCvCamera phoneCam;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        SampleMecanumDrive drive   = new SampleMecanumDrive(hardwareMap);
        Arm arm           = new Arm(hardwareMap);
        Intake intake     = new Intake(hardwareMap);

        // Open async and start streaming inside opened callback
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                phoneCam.setPipeline(detector);
            }

            @Override
            public void onError(int errorCode) {
               telemetry.addData("Failed to open camera due to error code", errorCode);
            }
        });

        // Do not delete this line of code, if you do the auto will not work
        PropDetectorRed.SkystoneLocation location = detector.getLocation();

        waitForStart();


        while (opModeIsActive()) {
            location = detector.getLocation();

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

            if (location == PropDetectorRed.SkystoneLocation.LEFT) {
                arm.extend(MED_EXTEND_POSITION);
                sleep(3000);
            } else if (location == PropDetectorRed.SkystoneLocation.RIGHT) {
                drive.turn(Math.toRadians(45));
                arm.extend(1500);
                sleep(3000);
            } else {
                drive.turn(Math.toRadians(-45));
                arm.extend(1500);
                sleep(3000);
            }

            // more robot logic...
            sleep(3000000);
        }
    }

}