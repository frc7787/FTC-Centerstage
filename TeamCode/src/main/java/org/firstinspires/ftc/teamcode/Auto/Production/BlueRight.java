package org.firstinspires.ftc.teamcode.Auto.Production;

// Roadrunner Imports
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

// SDK Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Team Code Imports
import org.firstinspires.ftc.teamcode.Subsytems.Intake;
import org.firstinspires.ftc.teamcode.Utility.PropDetectorPipeline;
import org.firstinspires.ftc.teamcode.Utility.PropDetectorPipeline.PropLocation;


@Autonomous(name = "Blue Right", group = "Blue")
public class BlueRight extends OpMode {

    public PropDetectorPipeline propDetector;
    public TrajectorySequence toSpikeStrip;
    public Intake intake;
    public SampleMecanumDrive drive;

    @Override public void init() {
        propDetector = new PropDetectorPipeline();
        intake       = new Intake(this);
        drive        = new SampleMecanumDrive(hardwareMap);

        Pose2d startPosition = new Pose2d(-36, 63, Math.toRadians(270.00d));

        toSpikeStrip = drive.trajectorySequenceBuilder(startPosition)
                .lineTo(new Vector2d(-36, 34))
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
