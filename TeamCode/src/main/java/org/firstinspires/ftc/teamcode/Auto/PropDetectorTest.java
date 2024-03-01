package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test - Prop Detector")
@Disabled
public class PropDetectorTest extends OpMode {
    private long lastButtonPress = 0;

    PropDetector detector;
    OpenCvCamera camera;

    @Override public void init() {
        detector = new PropDetector(PropColor.RED);

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
                camera.setPipeline(detector);
            }

            @Override public void onError(int errorCode) {
                telemetry.addData("Failed to open camera due to error code", errorCode);
                telemetry.update();
            }
        });
    }

    @Override public void init_loop() {
        telemetry.addData("Location", detector.getPropLocation());
        telemetry.addData("Detector Color", detector.propColor);
        telemetry.addLine("Press left bumper to swap prop detection colour between red and blue");

        if (gamepad1.left_bumper && (System.currentTimeMillis() - lastButtonPress) > 1000) {
            detector.swapColor();
            lastButtonPress = System.currentTimeMillis();
        }

        telemetry.update();
    }

    @Override public void loop() {}
}
