package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Blue Frontstage Parking Auto", group = "Parking Autos")

public class ParkingAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38,62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        waitForStart();

        TrajectorySequence bluefronttraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-38, 12))
                .build();
        TrajectorySequence bluefronttraj2 = drive.trajectorySequenceBuilder(bluefronttraj1.end())
                .lineTo(new Vector2d(60, 12))
                .build();


        if (isStopRequested()) return;

        while (!isStopRequested()) {

            drive.followTrajectorySequence(bluefronttraj1);
            drive.followTrajectorySequence(bluefronttraj2);

        }

    }
}