package org.firstinspires.ftc.teamcode.PATCHY;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue Frontstage Parking Auto", group = "Parking Autos")

public class BlueFrontAuto extends LinearOpMode {
//you fixed the turning on this one
    Pose2d visPose;
    Pose2d placePose;
    String auto;
    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.bluepropPipeline bluepropPipeline;
    DcMotorEx mtrI;


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
        telemetry.addData("blue Prop Position", bluepropPipeline.getPropPosition());
        telemetry.update();

        if (auto == "left"){
            visPose = new Pose2d(-36.00,35.00, Math.toRadians(270));
            placePose = new Pose2d(51.00, 42.50, Math.toRadians(180));

        }else if (auto == "right"){
            visPose = new Pose2d(-36.00,35.00, Math.toRadians(270));
            placePose = new Pose2d(51.00, 30.00, Math.toRadians(180));

        }else {
            visPose = new Pose2d(-52.00, 30.00, Math.toRadians(270));
            placePose = new Pose2d(51.00, 36.00, Math.toRadians(180));

        }

        mtrI =  hardwareMap.get(DcMotorEx.class, "mtrI");
        mtrI.setZeroPowerBehavior(BRAKE);
        mtrI.setDirection(DcMotor.Direction.REVERSE);
        mtrI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36,60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence bfTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36.00, 35.00))
                .build();

        //left trajs
        TrajectorySequence bfLeftProp1 = drive.trajectorySequenceBuilder(bfTraj1.end())
                .lineTo(new Vector2d(-32.00, 30.00))
                .turn(Math.toRadians(90))
                .waitSeconds(2)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence bfLeftProp2 = drive.trajectorySequenceBuilder(bfLeftProp1.end())
                .lineTo(new Vector2d(-36.00,35.00))
                .build();

        //right trajs
        TrajectorySequence bfRightProp1 = drive.trajectorySequenceBuilder(bfTraj1.end())
                .lineTo(new Vector2d(-40.00, 30.00))
                .turn(Math.toRadians(-90))
                .waitSeconds(2)
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence bfRightProp2 = drive.trajectorySequenceBuilder(bfRightProp1.end())
                .lineTo(new Vector2d(-36.00,35.00))
                .build();

        //center trajs
        TrajectorySequence bfCenterProp1 = drive.trajectorySequenceBuilder(bfTraj1.end())
                .lineTo(new Vector2d(-36.00, 30.00))
                .waitSeconds(2)
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




        waitForStart();


            drive.followTrajectorySequence(bfTraj1);

            if (auto == "left"){
                drive.followTrajectorySequence(bfLeftProp1);
                drive.followTrajectorySequence(bfLeftProp2);

            } else if (auto == "right") {
                drive.followTrajectorySequence(bfRightProp1);
                drive.followTrajectorySequence(bfRightProp2);

            } else {
                drive.followTrajectorySequence(bfCenterProp1);
                drive.followTrajectorySequence(bfCenterProp2);

            }

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
            drive.followTrajectorySequence(bfTraj4);

        }

    }