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
@Autonomous(name = "Red Backstage Parking Auto", group = "Parking Autos")

public class RedBackAuto extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(12,-62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence rbTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12.00, -35.00))
                .build();

        //left prop traj
        TrajectorySequence rbLeftProp1 = drive.trajectorySequenceBuilder(rbTraj1.end())
                .lineTo(new Vector2d(8.00, -30.00))
                .turn(Math.toRadians(-45))
                .waitSeconds(2)
                .turn(Math.toRadians(135))
                .build();
        TrajectorySequence rbLeftProp2 = drive.trajectorySequenceBuilder(rbLeftProp1.end())
                .lineTo(new Vector2d(52.00, -30.00))
                .waitSeconds(3)
                .build();

        //right prop traj
        TrajectorySequence rbRightProp1 = drive.trajectorySequenceBuilder(rbTraj1.end())
                .lineTo(new Vector2d(16.00, -30.00))
                .turn(Math.toRadians(90))
                .waitSeconds(2)
                .build();
        TrajectorySequence rbRightProp2 = drive.trajectorySequenceBuilder(rbRightProp1.end())
                .lineTo(new Vector2d(16.00, -42.50))
                .build();
        TrajectorySequence rbRightProp3 = drive.trajectorySequenceBuilder(rbRightProp2.end())
                .lineTo(new Vector2d(52.00, -42.50))
                .waitSeconds(3)
                .build();

        //center prop traj
        TrajectorySequence rbCenterProp1 = drive.trajectorySequenceBuilder(rbTraj1.end())
                .lineTo(new Vector2d(12.00, -30.00))
                .waitSeconds(2)
                .build();
        TrajectorySequence rbCenterProp2 = drive.trajectorySequenceBuilder(rbCenterProp1.end())
                .lineTo(new Vector2d(12.00, -36.00))
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence rbCenterProp3 = drive.trajectorySequenceBuilder(rbCenterProp2.end())
                .lineTo(new Vector2d(52.00, -36.00))
                .waitSeconds(3)
                .build();

        //final trajectories
        TrajectorySequence rbFinalTraj1 = drive.trajectorySequenceBuilder(placePose)
                .lineTo(new Vector2d(52.00, -62.00))
                .build();


        waitForStart();

        while (!isStopRequested()) {

            drive.followTrajectorySequence(rbTraj1);

            if (auto == "left"){
                drive.followTrajectorySequence(rbLeftProp1);
                drive.followTrajectorySequence(rbLeftProp2);
                placePose = rbRightProp2.end();
                drive.setPoseEstimate(placePose);

            } else if (auto == "right") {
                drive.followTrajectorySequence(rbRightProp1);
                drive.followTrajectorySequence(rbRightProp2);
                drive.followTrajectorySequence(rbRightProp3);
                placePose = rbRightProp3.end();
                drive.setPoseEstimate(placePose);

            } else {
                drive.followTrajectorySequence(rbCenterProp1);
                drive.followTrajectorySequence(rbCenterProp2);
                drive.followTrajectorySequence(rbCenterProp3);
                placePose = rbCenterProp3.end();
                drive.setPoseEstimate(placePose);

            }

            drive.followTrajectorySequence(rbFinalTraj1);

        }

    }
}
