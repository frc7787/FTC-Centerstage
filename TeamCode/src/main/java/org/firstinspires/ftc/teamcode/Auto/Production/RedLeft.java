package org.firstinspires.ftc.teamcode.Auto.Production;

// Roadrunner Imports
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

// SDK Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Team Code Imports
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.teamcode.Utility.PropDetectorPipeline;
import org.firstinspires.ftc.teamcode.Utility.PropDetectorPipeline.PropLocation;

@Autonomous(name = "Red Left", group = "Red")
public class RedLeft extends OpMode {

    public static PropDetectorPipeline propDetector;
    public static SampleMecanumDrive drive;
    public static Intake intake;
    public static TrajectorySequence toSpikeStrip;

    @Override public void init() {
        propDetector = new PropDetectorPipeline();
        drive        = new SampleMecanumDrive(hardwareMap);
        intake       = new Intake(this);

        Pose2d startPosition = new Pose2d(-36, -63, Math.toRadians(90.00d));

        toSpikeStrip = drive.trajectorySequenceBuilder(startPosition)
                .lineTo(new Vector2d(-36, -34))
                .build();
    }

    @Override public void loop() {
        PropLocation propLocation = propDetector.getLocation();

        drive.followTrajectorySequence(toSpikeStrip);

        switch (propLocation) {
            case CENTER:
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