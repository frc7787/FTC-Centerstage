package org.firstinspires.ftc.teamcode.Auto.Production;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorBlue;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropDetectorRed;
import org.firstinspires.ftc.teamcode.Subsytems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsytems.Elevator;
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import static org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name="Auto BLUE")
public class AutoModeBlue extends LinearOpMode {
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    PropDetectorBlue detector = new PropDetectorBlue();
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


        PropDetectorBlue.SkystoneLocation location = detector.getLocation();

        waitForStart();


        while (opModeIsActive()) {
            location = detector.getLocation();

            telemetry.addData("PROP LOCATION: ", location);
            telemetry.update();

            if (location == PropDetectorBlue.SkystoneLocation.RIGHT) {
                drive.strafe(DriveBase.StrafeDirections.RIGHT, 600, this);
                elevator.extend(1500);
                sleep(3000);
                intake.release();
                elevator.extend(0);
                drive.driveBackwards(500, this);
            } else if (location == PropDetectorBlue.SkystoneLocation.LEFT) {
                elevator.extend(MED_EXTEND_POSITION);
                sleep(3000);
                intake.release();
                elevator.extend(0);
                drive.driveBackwards(500, this);
            } else {
                drive.strafe(DriveBase.StrafeDirections.LEFT, 600, this);
                elevator.extend(1500);
                sleep(3000);
                intake.release();
                elevator.extend(0);
                drive.driveBackwards(500, this);
            }

            // more robot logic...
            sleep(3000000);
        }
    }

}