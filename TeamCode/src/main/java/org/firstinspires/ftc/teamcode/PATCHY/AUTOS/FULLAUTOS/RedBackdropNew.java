package org.firstinspires.ftc.teamcode.PATCHY.AUTOS.FULLAUTOS;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.redpropPipeline;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.PATCHY.hardwareCS;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Red, close to backdrop", group = "Final Autos")
public class RedBackdropNew extends LinearOpMode {

    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.redpropPipeline redpropPipeline;
    DcMotorEx mtrI;

    int state;

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareCS robot = new hardwareCS();
        robot.inithardware(hardwareMap);

        mtrI = hardwareMap.get(DcMotorEx.class, "mtrI");
        mtrI.setZeroPowerBehavior(BRAKE);
        mtrI.setDirection(DcMotor.Direction.FORWARD);
        mtrI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        telemetry.addData("Red Prop Position", redpropPipeline.getPropPosition());
        telemetry.update();

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startPose = new Pose2d(12.00, -62.75, Math.toRadians(90.00));


    // all of the center trajectories
    TrajectorySequence redBackdropCenterTrajsOutsidePark = drive.trajectorySequenceBuilder(startPose)
            .lineTo(new Vector2d(12.00, -34.50))
            .lineTo(new Vector2d(17.00, -34.50))
            .build();
    TrajectorySequence redBackdropCenterTrajsOutsidePark2 = drive.trajectorySequenceBuilder(redBackdropCenterTrajsOutsidePark.end())
            .lineTo(new Vector2d(51.50, -34.50))
            .build();
    TrajectorySequence redBackdropCenterTrajsOutsidePark3 = drive.trajectorySequenceBuilder(redBackdropCenterTrajsOutsidePark2.end())
            .lineTo(new Vector2d(48.00, -34.50))
            .build();



    //outside parking trajectory
    TrajectorySequence outsidePark = drive.trajectorySequenceBuilder(new Pose2d())
            .lineTo(new Vector2d(48.00, -60.00))
            .build();


        waitForStart();
        if (redpropPipeline.getPropPosition() == "left"){
            state = 10;
        } else if (redpropPipeline.getPropPosition() == "right") {
            state = 20;
        } else {
            state = 30;
        }

        if (isStopRequested()) return;

        switch (state){
            case (10):

                break;
            case (20):

                break;
            case (30):
                drive.followTrajectorySequence(redBackdropCenterTrajsOutsidePark);
                drive.followTrajectorySequence(redBackdropCenterTrajsOutsidePark2);
                drive.followTrajectorySequence(redBackdropCenterTrajsOutsidePark3);
                break;
        }

        //if switch == high or whatever, we'll put this in
        //else
        //inside park trajectory

    }
}
