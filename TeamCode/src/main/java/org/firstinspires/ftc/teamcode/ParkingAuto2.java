package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TrackingWheelForwardOffsetTuner;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Red Frontstage Parking Auto", group = "Parking Autos")

public class ParkingAuto2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38,-61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence redfronttraj1 = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(-38.00, -12.00))
                    .build();
            TrajectorySequence redfronttraj2 = drive.trajectorySequenceBuilder(redfronttraj1.end())
                    .lineTo(new Vector2d(60.00, -12.00))
                    .build();


            drive.followTrajectorySequence(redfronttraj1);
            drive.followTrajectorySequence(redfronttraj2);
        }

    }
}
