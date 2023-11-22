package org.firstinspires.ftc.teamcode.Auto.Production;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorBlue;
import org.firstinspires.ftc.teamcode.Subsytems.Arm;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import static org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name="Auto BLUE")
public class AutoModeBlue extends LinearOpMode {

    PropDetectorBlue detector = new PropDetectorBlue();
    OpenCvCamera phoneCam;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        DriveBase drive = new DriveBase(hardwareMap);
        Arm arm         = new Arm(hardwareMap);
        Intake intake   = new Intake(hardwareMap);

        // Open async and start streaming inside opened callback
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                phoneCam.setPipeline(detector);
            }

            @Override
            public void onError(int errorCode) {
               telemetry.addData("Failed to open camera with error code", errorCode);
               telemetry.update();
            }
        });

        // Don't delete this line of code, Android Studio says it's redundant but it isn't
        PropDetectorBlue.SkystoneLocation location = detector.getLocation();

        waitForStart();

        while (opModeIsActive()) {
            location = detector.getLocation();

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

            if (location == PropDetectorBlue.SkystoneLocation.RIGHT) {
                drive.strafe(DriveBase.StrafeDirections.RIGHT);
                sleep(600);
                arm.extend(1500);
                sleep(3000);
                intake.intake();
                arm.extend(0);
                drive.driveBackwards();
                sleep(500);
            } else if (location == PropDetectorBlue.SkystoneLocation.LEFT) {
                arm.extend(MED_EXTEND_POSITION);
                sleep(3000);
                intake.intake();
                arm.extend(0);
                drive.driveBackwards();
                sleep(500);
            } else {
                drive.strafe(DriveBase.StrafeDirections.LEFT);
                sleep(600);
                arm.extend(1500);
                sleep(3000);
                intake.intake();
                arm.extend(0);
                drive.driveBackwards();
                sleep(500);
            }

            // more robot logic...
            sleep(3000000);
        }
    }

}