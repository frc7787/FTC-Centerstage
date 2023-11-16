package org.firstinspires.ftc.teamcode.Auto.Production;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.teamcode.Auto.Utility.RedPropDetector;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Red Right", group = "Red")
public class RedRight extends OpMode {

    public static RedPropDetector propDetector;
    public static SampleMecanumDrive drive;
    public static Intake intake;
    public static TrajectorySequence toSpikeStrip;

    @Override public void init() {
        propDetector = new RedPropDetector();
        drive        = new SampleMecanumDrive(hardwareMap);
        intake       = new Intake(this);

        Pose2d startPosition = new Pose2d(12, -63, Math.toRadians(90.00d));

        toSpikeStrip = drive.trajectorySequenceBuilder(startPosition)
                .lineTo(new Vector2d(11, -34))
                .build();
    }

    @Override public void loop() {
        PropLocation propLocation = propDetector.getLocation();

        drive.followTrajectorySequence(toSpikeStrip);

        switch (propLocation) {
            case CENTER: // None means that the prop is in the center
                intake.release();
            case LEFT:
                drive.turn(90.00d);
                intake.release();
            case RIGHT:
                drive.turn(-90.00d);
                intake.release();
        }
    }
}