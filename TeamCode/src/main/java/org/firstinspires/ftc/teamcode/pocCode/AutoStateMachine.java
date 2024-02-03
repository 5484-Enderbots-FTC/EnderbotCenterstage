package org.firstinspires.ftc.teamcode.pocCode;


import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PATCHY.bluepropPipeline;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardwareCS;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Autostate Blue Front", group = "Parking Autos")

public class AutoStateMachine extends LinearOpMode {
    //you fixed the turning on this one
    Pose2d visPose;
    Pose2d placePose;
    String auto;
    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.bluepropPipeline bluepropPipeline;

    DcMotor mtrI;
    private int state = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        hardwareCS robot = new hardwareCS();
        robot.inithardware(hardwareMap);

        mtrI = robot.mtrI;
//predefining positions for purple and yellow pixel placement because we always know where it is going to end
        if (auto == "left") {
            visPose = new Pose2d(-36.00, 35.00, Math.toRadians(270));
            placePose = new Pose2d(51.00, 42.50, Math.toRadians(180));
            state = 0;

        } else if (auto == "right") {
            visPose = new Pose2d(-36.00, 35.00, Math.toRadians(270));
            placePose = new Pose2d(51.00, 30.00, Math.toRadians(180));
            state = 10;

        } else {
            visPose = new Pose2d(-52.00, 30.00, Math.toRadians(270));
            placePose = new Pose2d(51.00, 36.00, Math.toRadians(180));
            state = 20;

        }
//this is the robot starting from right far back
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //defining movement to placement zone for purple pixel
        TrajectorySequence bfTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36.00, 35.00))
                .build();

        //left trajs
        TrajectorySequence bfLeftProp1 = drive.trajectorySequenceBuilder(bfTraj1.end())
                .lineTo(new Vector2d(-32.00, 30.00))
                .turn(Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    mtrI.setPower(0.5);

                })
                .waitSeconds(.5)
                .addDisplacementMarker(() -> {
                    mtrI.setPower(0);

                })
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence bfLeftProp2 = drive.trajectorySequenceBuilder(bfLeftProp1.end())
                .lineTo(new Vector2d(-36.00, 35.00))
                .build();

        //right trajs
        TrajectorySequence bfRightProp1 = drive.trajectorySequenceBuilder(bfTraj1.end())
                .lineTo(new Vector2d(-40.00, 30.00))
                .turn(Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    mtrI.setPower(1.0);

                })
                .waitSeconds(1.0)
                .addDisplacementMarker(() -> {
                    mtrI.setPower(0);

                })
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence bfRightProp2 = drive.trajectorySequenceBuilder(bfRightProp1.end())
                .lineTo(new Vector2d(-36.00, 35.00))
                .build();

        //center trajs
        TrajectorySequence bfCenterProp1 = drive.trajectorySequenceBuilder(bfTraj1.end())
                .lineTo(new Vector2d(-36.00, 30.00))
                .addDisplacementMarker(() -> {
                    mtrI.setPower(0.5);

                })
                .waitSeconds(.5)
                .addDisplacementMarker(() -> {
                    mtrI.setPower(0);

                })
                .build();
        TrajectorySequence bfCenterProp2 = drive.trajectorySequenceBuilder(bfCenterProp1.end())
                .lineTo(new Vector2d(-52.00, 30.00))
                .build();

        //everyone does
        TrajectorySequence bfTraj2 = drive.trajectorySequenceBuilder(visPose)
                .lineTo(new Vector2d(-36.00, 10.00))
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence bfTraj3 = drive.trajectorySequenceBuilder(bfTraj2.end())
                .lineTo(new Vector2d(36.00, 10.00))
                .build();

        //left tag
        TrajectorySequence bfLeftProp3 = drive.trajectorySequenceBuilder(bfTraj3.end())
                .lineTo(new Vector2d(36.00, 42.50))
                .build();
        TrajectorySequence bfLeftProp4 = drive.trajectorySequenceBuilder(bfLeftProp3.end())
                .lineTo(new Vector2d(51.00, 42.50))
                .waitSeconds(3)
                .build();

        //right tag
        TrajectorySequence bfRightProp3 = drive.trajectorySequenceBuilder(bfTraj3.end())
                .lineTo(new Vector2d(36.00, 30.00))
                .build();
        TrajectorySequence bfRightProp4 = drive.trajectorySequenceBuilder(bfRightProp3.end())
                .lineTo(new Vector2d(51.00, 30.00))
                .waitSeconds(3)
                .build();

        //center tag
        TrajectorySequence bfCenterProp3 = drive.trajectorySequenceBuilder(bfTraj3.end())
                .lineTo(new Vector2d(36.00, 36.00))
                .build();
        TrajectorySequence bfCenterProp4 = drive.trajectorySequenceBuilder(bfCenterProp3.end())
                .lineTo(new Vector2d(51.00, 36.00))
                .waitSeconds(3)
                .build();

        //finale
        TrajectorySequence bfTraj4 = drive.trajectorySequenceBuilder(placePose)
                .lineTo(new Vector2d(52.00, 62.00))
                .build();


        while (opModeInInit()) {
            robot.initWebcamBlue();
            auto = robot.getOutString();

        }

        waitForStart();


        drive.followTrajectorySequence(bfTraj1);

        if (auto == "left") {
            drive.followTrajectorySequence(bfLeftProp1);
            drive.followTrajectorySequence(bfLeftProp2);

        } else if (auto == "right") {
            drive.followTrajectorySequence(bfRightProp1);
            drive.followTrajectorySequence(bfRightProp2);

        } else {
            drive.followTrajectorySequence(bfCenterProp1);
            drive.followTrajectorySequence(bfCenterProp2);

        }
/*
            drive.setPoseEstimate(visPose);
            drive.followTrajectorySequence(bfTraj2);
            drive.followTrajectorySequence(bfTraj3);

            if (auto == "left"){
                drive.followTrajectorySequence(bfLeftProp3);
                drive.followTrajectorySequence(bfLeftProp4);

            } else if (auto == "right") {
                drive.followTrajectorySequence(bfRightProp3);
                drive.followTrajectorySequence(bfRightProp4);

            } else {
                drive.followTrajectorySequence(bfCenterProp3);
                drive.followTrajectorySequence(bfCenterProp4);

            }

            drive.setPoseEstimate(placePose);
            drive.followTrajectorySequence(bfTraj4); */

    }
}

