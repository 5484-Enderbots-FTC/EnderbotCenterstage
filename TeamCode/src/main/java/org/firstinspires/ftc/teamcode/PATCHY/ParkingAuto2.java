package org.firstinspires.ftc.teamcode.PATCHY;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "Red Frontstage Parking Auto", group = "Parking Autos")

public class ParkingAuto2 extends LinearOpMode {

    Pose2d visPose;
    Pose2d placePose;
    String auto;
    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.redpropPipeline redpropPipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        redpropPipeline = new redpropPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(redpropPipeline)
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

        auto = redpropPipeline.getPropPosition();
        telemetry.addData("Red Prop Position", redpropPipeline.getPropPosition());
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence redfronttraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36.00, -37.00))
                .build();

        //left trajectories
        TrajectorySequence rfLeftTraj1 = drive.trajectorySequenceBuilder(redfronttraj1.end())
                .lineTo(new Vector2d(-40.00, -33.00))
                .turn(Math.toRadians(-45))
                .waitSeconds(2)
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence rfLeftTraj2 = drive.trajectorySequenceBuilder(rfLeftTraj1.end())
                .lineTo(new Vector2d(-36.00, -37.00))
                .build();

        //right trajectories
        TrajectorySequence rfRightTraj1 = drive.trajectorySequenceBuilder(redfronttraj1.end())
                .lineTo(new Vector2d(-31.00, -33.00))
                .turn(Math.toRadians(45))
                .waitSeconds(2)
                .turn(Math.toRadians(-45))
                .build();

        TrajectorySequence rfRightTraj2 = drive.trajectorySequenceBuilder(rfRightTraj1.end())
                .lineTo(new Vector2d(-36.00, -37.00))
                .build();

        //center trajectories
        TrajectorySequence rfCenterTraj1 = drive.trajectorySequenceBuilder(redfronttraj1.end())
                .lineTo(new Vector2d(-36.00, -30.00))
                .build();

        //final trajectories
        TrajectorySequence rfFinalTraj1 = drive.trajectorySequenceBuilder(visPose)
                .lineTo(new Vector2d(-36.00, -10.00))
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence rfFinalTraj2 = drive.trajectorySequenceBuilder(rfFinalTraj1.end())
                .lineTo(new Vector2d(36.00, -10.00))
                .build();

        //traj to place pixels
        TrajectorySequence rfTagLeft = drive.trajectorySequenceBuilder(rfFinalTraj2.end())
            .lineTo(new Vector2d(52.00, -29.50))
            .waitSeconds(2)
            .build();

        TrajectorySequence rfTagCenter = drive.trajectorySequenceBuilder(rfFinalTraj2.end())
            .lineTo(new Vector2d(52.00, -36.00))
            .waitSeconds(2)
            .build();

        TrajectorySequence rfTagRight = drive.trajectorySequenceBuilder(rfFinalTraj2.end())
            .lineTo(new Vector2d(52.00, -42.25))
            .waitSeconds(2)
            .build();

        //should be final final
        TrajectorySequence rfUltimatum = drive.trajectorySequenceBuilder(placePose)
            .lineTo(new Vector2d(50.00, -12.00))
            .build();






        waitForStart();
        while (opModeIsActive()) {

            while (!isStopRequested()) {

                drive.followTrajectorySequence(redfronttraj1);

                //place pixel
                if (auto == "left") {
                    //left side traj
                    drive.followTrajectorySequence(rfLeftTraj1);
                    drive.followTrajectorySequence(rfLeftTraj2);
                    visPose = rfLeftTraj2.end();

                } else if (auto == "right") {
                    //right side traj
                    drive.followTrajectorySequence(rfRightTraj1);
                    drive.followTrajectorySequence(rfRightTraj2);
                    visPose = rfRightTraj2.end();

                } else {
                    //center traj
                    drive.followTrajectorySequence(rfCenterTraj1);
                    visPose = rfCenterTraj1.end();
                }

                //final trajes
                drive.followTrajectorySequence(rfFinalTraj1);
                drive.followTrajectorySequence(rfFinalTraj2);

                if (auto == "left") {
                    //left tag traj
                    drive.followTrajectorySequence(rfTagLeft);
                    placePose = rfTagLeft.end();

                } else if (auto == "right") {
                    //right side traj
                    drive.followTrajectorySequence(rfTagRight);
                    placePose = rfTagRight.end();

                } else {
                    //center traj
                    drive.followTrajectorySequence(rfTagCenter);
                    placePose = rfTagCenter.end();

                }

                drive.followTrajectorySequence(rfUltimatum);

            }

        }
    }
}
