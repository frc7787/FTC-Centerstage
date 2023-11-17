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
import org.firstinspires.ftc.teamcode.Auto.Utility.BluePropDetector;
import org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation;

@Autonomous(name = "Blue Left", group = "Blue")
public class BlueLeft extends OpMode {

    public BluePropDetector propDetector;
    public SampleMecanumDrive drive;
    public Intake intake;
    public TrajectorySequence toSpikeStrip;

    @Override public void init() {
        propDetector = new BluePropDetector();
        drive        = new SampleMecanumDrive(hardwareMap);
        intake       = new Intake(this);

        Pose2d startPosition = new Pose2d(12, 63, Math.toRadians(270.00d));

        toSpikeStrip = drive.trajectorySequenceBuilder(startPosition)
                .lineTo(new Vector2d(12, 34))
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