package org.firstinspires.ftc.teamcode.Auto.Utility;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTag {
    AprilTagProcessor aprilTag;

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
