package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class DriveToPropHelper {

    public static void DriveToProp(DcMotor f_l_motor, DcMotor f_r_motor, DcMotor b_l_motor, DcMotor b_r_motor, PropDetectorPipeline detector, OpenCvCamera phoneCam, OpMode opMode) {
        // Create camera instance
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        OpenCvCamera finalPhoneCam = phoneCam;
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalPhoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                finalPhoneCam.setPipeline(detector);
            }

            @Override
            public void onError(int errorCode) {
                // Handle error
            }
        });

        // Rest of your DriveToProp logic
        PropDetectorPipeline.PropLocation location = detector.getLocation();
        double detectionMotorPower = 0.2;

        if (location == PropDetectorPipeline.PropLocation.CENTER) {
            f_l_motor.setPower(0);
            f_r_motor.setPower(0);
            b_l_motor.setPower(0);
            b_r_motor.setPower(0);
            opMode.telemetry.addLine("STOP MOVING MY CHILD");
        } else if (location == PropDetectorPipeline.PropLocation.RIGHT) {
            f_l_motor.setPower(0);
            f_r_motor.setPower(detectionMotorPower);
            b_l_motor.setPower(0);
            b_r_motor.setPower(detectionMotorPower);
            opMode.telemetry.addLine("RIGHT MY CHILD");
        } else if (location == PropDetectorPipeline.PropLocation.LEFT) {
            f_l_motor.setPower(detectionMotorPower);
            f_r_motor.setPower(0);
            b_l_motor.setPower(detectionMotorPower);
            b_r_motor.setPower(0);
            opMode.telemetry.addLine("LEFT MY CHILD");
        }
    }
}