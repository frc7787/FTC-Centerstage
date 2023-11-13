package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.teamcode.Utility.PropDetectorPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Left", group = "Blue")
public class BlueLeft extends OpMode {

    public static PropDetectorPipeline propDetector;
    public static SampleMecanumDrive drive;
    public static Intake intake;
    public static TrajectorySequence trajectory;

    @Override public void init() {
        propDetector = new PropDetectorPipeline();
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(this);

        trajectory = drive.trajectorySequenceBuilder(new Pose2d(12.06, 62.32, Math.toRadians(270.00)))
                .lineTo(new Vector2d(12.06, 34.16))
                .build();
    }

    @Override public void loop() {
        if (propDetector.getLocation() == PropDetectorPipeline.PropLocation.NONE) {
            drive.followTrajectorySequence(trajectory);
            intake.release();
        }
        if (propDetector.getLocation() == PropDetectorPipeline.PropLocation.LEFT) {
            drive.followTrajectorySequence(trajectory);
            drive.turn(90);
            intake.release();
        }
        if (propDetector.getLocation() == PropDetectorPipeline.PropLocation.RIGHT) {
            drive.followTrajectorySequence(trajectory);
            drive.turn(-90);
            intake.release();
        }
    }
}
