package org.firstinspires.ftc.teamcode.Auto.Utility;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTag {
    AprilTagProcessor aprilTag;

    /**
     * Initialize the AprilTag processor.
     */
    public VisionPortal initAprilTag(WebcamName webcam1, WebcamName webcam2) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Create the vision portal by using a builder.
        return new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTag)
                .build();
    }


    public void centerOnAprilTag(AprilTagDetection detection) {
        double x =  detection.rawPose.x;
        double y =  detection.rawPose.z;
        double z = -detection.rawPose.y;

        AngleUnit outputUnitsAngle = null;
        Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, outputUnitsAngle);

        double yaw = -rot.firstAngle;
        double roll = rot.thirdAngle;
        double pitch = rot.secondAngle;

        double range = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
        double bearing = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-detection.ftcPose.x, detection.ftcPose.y));
        double elevation = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(detection.ftcPose.z, detection.ftcPose.y));


    }


    public AprilTagDetection detectAprilTag(int id) {
        AprilTagDetection output = null;

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.metadata != null) {
                if ((id < 0) || (detection.id == id)) {
                    output = detection;
                }
            }
        }

        return output;
    }
}
