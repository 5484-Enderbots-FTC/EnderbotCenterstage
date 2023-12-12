package org.firstinspires.ftc.teamcode.PATCHY;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Red Backstage Parking Auto", group = "Parking Autos")

public class ParkingAuto4 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(16,-62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence redbacktraj1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(60.00, -62.00), Math.toRadians(0.00))
                .build();


        while (!isStopRequested()) {

            drive.followTrajectorySequence(redbacktraj1);
        }

    }
}
