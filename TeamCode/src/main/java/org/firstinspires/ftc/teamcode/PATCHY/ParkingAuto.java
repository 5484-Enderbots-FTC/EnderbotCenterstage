package org.firstinspires.ftc.teamcode.PATCHY;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;

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
                .lineTo(new Vector2d(-38, 10))
                .build();
        TrajectorySequence bluefronttraj2 = drive.trajectorySequenceBuilder(bluefronttraj1.end())
                .lineTo(new Vector2d(58, 10))
                .build();


        if (isStopRequested()) return;

        while (!isStopRequested()) {

            drive.followTrajectorySequence(bluefronttraj1);
            drive.followTrajectorySequence(bluefronttraj2);

        }

    }
}