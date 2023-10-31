package org.firstinspires.ftc.teamcode.Auto;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto: SkyStone Detector")
public class AutoMode extends LinearOpMode {
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    SkystoneDetector detector = new SkystoneDetector(width);
    OpenCvCamera phoneCam;

    public static DcMotor f_l_motor, f_r_motor, b_l_motor, b_r_motor;

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

        waitForStart();

        f_l_motor = hardwareMap.get(DcMotor.class, "Front Left Motor");
        f_r_motor = hardwareMap.get(DcMotor.class, "Front Right Motor");
        b_l_motor = hardwareMap.get(DcMotor.class, "Back Left Motor");
        b_r_motor = hardwareMap.get(DcMotor.class, "Back Right Motor");

        f_l_motor.setDirection(DcMotor.Direction.REVERSE);
        b_l_motor.setDirection(DcMotor.Direction.REVERSE);

        while (opModeIsActive()) {
            SkystoneDetector.SkystoneLocation location = detector.getLocation();

            telemetry.addData("Kai is just better", location);

            if (location == SkystoneDetector.SkystoneLocation.NONE) {
                f_l_motor.setPower(0);
                f_r_motor.setPower(0);
                b_l_motor.setPower(0);
                b_r_motor.setPower(0);
                telemetry.addLine("STOP MOVING MY CHILD");
            } else if (location == SkystoneDetector.SkystoneLocation.RIGHT) {
                f_l_motor.setPower(0.5);
                f_r_motor.setPower(0);
                b_l_motor.setPower(0.5);
                b_r_motor.setPower(0);
                telemetry.addLine("RIGHT MY CHILD");
            } else if (location == SkystoneDetector.SkystoneLocation.LEFT) {
                f_l_motor.setPower(0);
                f_r_motor.setPower(0.5);
                b_l_motor.setPower(0);
                b_r_motor.setPower(0.5);
                telemetry.addLine("LEFT MY CHILD");
            }

            // more robot logic...
            telemetry.update();
        }
    }

}