package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.util.Encoder;
import static org.firstinspires.ftc.teamcode.RoadRunner.util.Encoder.Direction.REVERSE;

import java.util.Arrays;
import java.util.List;

@Config
public class TrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    // --------------------------------------------------------------------------
    // NOTE
    // Any whole number of the double type should have a .0 added at the end to prevent integer division
    // --------------------------------------------------------------------------

    public static double TICKS_PER_REV = 2000.0d;
    public static double WHEEL_RADIUS  = 0.944882d; // in
    //public static double GEAR_RATIO = 1.0d;  // output (wheel) speed / input (encoder) speed
    public static double SLIP_RATIO    = 1.0d; // Variable to account for the wheels slipping on the mat

    public static double LATERAL_DISTANCE = 9.147d; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET   = 7.32d; // in; offset of the lateral wheel

    private final Encoder leftEncoder, rightEncoder, frontEncoder;

    private final List<Integer> lastEncPositions, lastEncVels;

    public TrackingWheelLocalizer(@NonNull HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0),  // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder  = new Encoder(hardwareMap.get(DcMotorEx.class, "BackLeftDriveMotor"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BackRightDriveMotor"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "IntakeMotor"));
    }

    public void init() {
       leftEncoder.setDirection(REVERSE);
       frontEncoder.setDirection(REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * SLIP_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull @Override public List<Double> getWheelPositions() {
        int leftPos  = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();

        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos),
                encoderTicksToInches(rightPos),
                encoderTicksToInches(frontPos)
        );
    }

    @NonNull @Override public List<Double> getWheelVelocities() {
        int leftVel  = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel),
                encoderTicksToInches(rightVel),
                encoderTicksToInches(frontVel)
        );
    }
}
