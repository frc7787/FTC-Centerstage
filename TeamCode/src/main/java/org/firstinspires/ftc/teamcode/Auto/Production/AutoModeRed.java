package org.firstinspires.ftc.teamcode.Auto.Production;

import static org.firstinspires.ftc.teamcode.Constants.HIGH_EXTEND_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.MED_EXTEND_POSITION;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorRed;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Elevator;
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
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera2Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        DriveBase drive = new DriveBase(hardwareMap);
        Elevator elevator = new Elevator(this);
        Intake intake = new Intake(this);

        // Open async and start streaming inside opened callback
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                phoneCam.setPipeline(detector);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        PropDetectorRed.SkystoneLocation location = detector.getLocation();

        waitForStart();


        while (opModeIsActive()) {
            location = detector.getLocation();

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

            if (location == PropDetectorRed.SkystoneLocation.LEFT) {
                drive.strafe(DriveBase.StrafeDirections.LEFT, 600, this);
                elevator.extend(MED_EXTEND_POSITION);
                intake.release();
            } else if (location == PropDetectorRed.SkystoneLocation.RIGHT) {
                elevator.extend(HIGH_EXTEND_POSITION);
                intake.release();
            } else {
                drive.strafe(DriveBase.StrafeDirections.RIGHT, 600, this);
                elevator.extend(MED_EXTEND_POSITION);
                intake.release();
            }

            // more robot logic...
            sleep(3000000);
        }
    }

}