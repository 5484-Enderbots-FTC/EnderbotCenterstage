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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence redfronttraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-38.00, -10.00))
                .build();
        TrajectorySequence redfronttraj2 = drive.trajectorySequenceBuilder(redfronttraj1.end())
                .lineTo(new Vector2d(58.00, -10.00))
                .build();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Red Prop Position", redpropPipeline.getPropPosition());
            telemetry.update();

            while (!isStopRequested()) {

                drive.followTrajectorySequence(redfronttraj1);
                drive.followTrajectorySequence(redfronttraj2);
            }

        }
    }
}
