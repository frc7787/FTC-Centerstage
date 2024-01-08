package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Utility.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "April Tag Testing")
public class AprilTagTesting extends LinearOpMode {

    AprilTag aprilTagDetection = new AprilTag();

    WebcamName webcam1, webcam2;

    AprilTagDetection aprilTag;


    @Override
    public void runOpMode() throws InterruptedException {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        aprilTagDetection.initAprilTag(webcam1, webcam2);

        while (opModeIsActive()) {
            aprilTag = aprilTagDetection.detectAprilTag(10);

            telemetry.addLine(String.format("\n==== (ID %d) %s", aprilTag.id, aprilTag.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", aprilTag.ftcPose.x, aprilTag.ftcPose.y, aprilTag.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", aprilTag.ftcPose.pitch, aprilTag.ftcPose.roll, aprilTag.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", aprilTag.ftcPose.range, aprilTag.ftcPose.bearing, aprilTag.ftcPose.elevation));
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", aprilTag.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", aprilTag.center.x, aprilTag.center.y));

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            telemetry.update();
        }
    }
}
