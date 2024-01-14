package org.firstinspires.ftc.teamcode.PATCHY;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue Backstage Parking Auto", group = "Parking Autos")

public class BlueBackAuto extends LinearOpMode {
//turns fixed on this one
    Pose2d placePose;
    String auto;
    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.bluepropPipeline bluepropPipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        bluepropPipeline = new bluepropPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(bluepropPipeline)
                .build();

        //portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d"));
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        auto = bluepropPipeline.getPropPosition();
        telemetry.addData("Blue Prop Position", bluepropPipeline.getPropPosition());
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12,62,Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence bbTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12.00, 35.00))
                .build();

        //left trajectories
        TrajectorySequence blueLeftProp1 = drive.trajectorySequenceBuilder(bbTraj1.end())
            .lineTo(new Vector2d(18.00, 30.00))
            .turn(Math.toRadians(90))
            .waitSeconds(2)
            .turn(Math.toRadians(180))
            .build();
        TrajectorySequence blueLeftProp2 = drive.trajectorySequenceBuilder(blueLeftProp1.end())
            .lineTo(new Vector2d(18.00, 42.50))
            .build();
        TrajectorySequence blueLeftProp3 = drive.trajectorySequenceBuilder(blueLeftProp2.end())
            .lineTo(new Vector2d(52.00, 42.50))
            .waitSeconds(3)
            .build();


        //center trajectories
        TrajectorySequence blueCenterProp1 = drive.trajectorySequenceBuilder(bbTraj1.end())
            .lineTo(new Vector2d(12.00, 30.00))
            .waitSeconds(2)
            .build();
        TrajectorySequence blueCenterProp2 = drive.trajectorySequenceBuilder(blueCenterProp1.end())
                .lineTo(new Vector2d(12.00, 36.00))
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence blueCenterProp3 = drive.trajectorySequenceBuilder(blueCenterProp2.end())
                .lineTo(new Vector2d(52.00, 36.00))
                .build();


        //right trajectories
        TrajectorySequence blueRightProp1 = drive.trajectorySequenceBuilder(bbTraj1.end())
                .lineTo(new Vector2d(8.00, 30.00))
                .turn(Math.toRadians(-90))
                .waitSeconds(2)
                .build();

        TrajectorySequence blueRightProp2 = drive.trajectorySequenceBuilder(blueRightProp1.end())
                .lineTo(new Vector2d(52.00, 30.00))
                .waitSeconds(3)
                .build();

        //final trajectories
        TrajectorySequence bbFinalTraj1 = drive.trajectorySequenceBuilder(placePose)
                .lineTo(new Vector2d(52.00, 62.00))
                .build();


        waitForStart();

        while (!isStopRequested()) {
            
            drive.followTrajectorySequence(bbTraj1);

            if (auto == "left"){
                drive.followTrajectorySequence(blueLeftProp1);
                drive.followTrajectorySequence(blueLeftProp2);
                drive.followTrajectorySequence(blueLeftProp3);
                placePose = blueLeftProp3.end();

            } else if (auto == "right") {
                drive.followTrajectorySequence(blueRightProp1);
                drive.followTrajectorySequence(blueRightProp2);
                placePose = blueRightProp2.end();

            } else {
                drive.followTrajectorySequence(blueCenterProp1);
                drive.followTrajectorySequence(blueCenterProp2);
                drive.followTrajectorySequence(blueCenterProp3);
                placePose = blueCenterProp3.end();

            }

            drive.setPoseEstimate(placePose);
            drive.followTrajectorySequence(bbFinalTraj1);

        }

    }
}
