package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.teamcode.Utility.PropDetectorPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Right", group = "Blue")
public class BlueRight extends OpMode {

    public static PropDetectorPipeline propDetector;
    public static TrajectorySequence trajectory;
    public static Intake intake;
    public static SampleMecanumDrive drive;

    @Override public void init() {
        propDetector = new PropDetectorPipeline();

        intake = new Intake(this);

        drive = new SampleMecanumDrive(hardwareMap);

        trajectory = drive.trajectorySequenceBuilder(new Pose2d(-36.18, 62.76, Math.toRadians(270.00)))
                .lineTo(new Vector2d(-36.32, 34.01))
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
