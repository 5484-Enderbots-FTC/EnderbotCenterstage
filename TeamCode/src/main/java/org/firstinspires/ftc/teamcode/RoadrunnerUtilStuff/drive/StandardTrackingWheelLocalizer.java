package org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PATCHY.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.688975; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.51863426391601; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -9; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Double> lastEncPositions;
    private List<Double> lastEncVels;

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.02360548; // Multiplier in the Y direction

    SparkFunOTOS myOtos;

// :)
    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Double> lastTrackingEncPositions, List<Double> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "mtrFL"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "mtrFR"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "mtrBL"));

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        /*int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();*/

        double otosPosX = myOtos.getPosition().x;
        double otosPosY = myOtos.getPosition().y;

        lastEncPositions.clear();

        lastEncPositions.add(otosPosX);
        lastEncPositions.add(otosPosY);
        /*lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);*/

        return Arrays.asList(
                otosPosX * X_MULTIPLIER,
                otosPosY * Y_MULTIPLIER
                /*encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER*/
        );
    }

    // @NonNull
   // @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        double otosVelX = myOtos.getVelocity().x;
        double otosVelY = myOtos.getVelocity().y;

        lastEncVels.clear();
        lastEncVels.add(otosVelX);
        lastEncVels.add(otosVelY);
        /*lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);*/

        return Arrays.asList(
                /*encoderTicksToInches(leftVel) * X_MULTIPLIER,
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER*/
                otosVelX * X_MULTIPLIER,
                otosVelY * Y_MULTIPLIER
        );
    }
}
