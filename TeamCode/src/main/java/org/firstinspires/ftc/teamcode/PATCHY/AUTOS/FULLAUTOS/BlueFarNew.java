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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.bluepropPipeline;
import org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.redpropPipeline;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.PATCHY.hardwareCS;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue, Far From Backdrop", group = "Far Autos")
public class BlueFarNew extends LinearOpMode {

    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.bluepropPipeline bluepropPipeline;
    DcMotorEx mtrI;

    private String auto;
    Servo gripper;

    int state;
    Pose2d parkPose;
    ElapsedTime lifttime;

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareCS robot = new hardwareCS();
        robot.inithardware(hardwareMap);

        gripper = hardwareMap.get(Servo.class, "svrGrip");

        mtrI =  hardwareMap.get(DcMotorEx.class, "mtrI");
        mtrI.setZeroPowerBehavior(BRAKE);
        mtrI.setDirection(DcMotor.Direction.FORWARD);
        mtrI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        Pose2d startPose = new Pose2d(-36,62.75, Math.toRadians(270));

        //one universal trajectory
        TrajectorySequence redBackdropUniversalTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36.00, 36.50))
                .addTemporalMarker(0, () -> {
                    robot.intakeRight.setPosition(.94);
                    gripper.setPosition(.6);
                })
                .addTemporalMarker(0.3, () -> {
                    robot.intakeLeft.setPosition(.027);
                })
                .build();


        //is it on the right?
        TrajectorySequence redBackdropRightTrajs1 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    robot.intakeRight.setPosition(.94);
                    gripper.setPosition(.6);
                })
                .addTemporalMarker(0.3, () -> {
                    robot.intakeLeft.setPosition(.027);
                })
                .addTemporalMarker(1.5, () -> {
                    mtrI.setPower(.6);
                    gripper.setPosition(.57);
                })
                .waitSeconds(1.5)
                .turn(Math.toRadians(180) - 1e+6)
                .build();

        TrajectorySequence redBackdropRightTrajs2 = drive.trajectorySequenceBuilder(redBackdropRightTrajs1.end())
                .lineTo(new Vector2d(-40.00, 60.25))
                .addTemporalMarker(0, () -> {
                    mtrI.setPower(0);
                    gripper.setPosition(.57);
                })
                .build();

        TrajectorySequence insertRight = drive.trajectorySequenceBuilder(redBackdropRightTrajs2.end())
                .lineTo(new Vector2d(-12.50, 60.25))
                .lineTo(new Vector2d(-12.50, 35.00))
                .lineTo(new Vector2d(50.00, 35.00))
                .build();

        TrajectorySequence redBackdropRightTrajs3 = drive.trajectorySequenceBuilder(insertRight.end())
                .lineTo(new Vector2d(50.00, 27.00))
                .addTemporalMarker(0, () -> {
                    robot.mtrLift.setVelocity(1000);
                    gripper.setPosition(.57);
                })
                .addTemporalMarker(.7, () -> {
                    robot.mtrLift.setVelocity(0);
                    robot.armSwing.setPosition(1.0);
                })
                .addTemporalMarker(2, () -> {
                    gripper.setPosition(.32);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(46, 27))
                .build();

        //center trajectories
        //center should be good
        TrajectorySequence RedBackDropCenterTrajs1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-40, 37.5))
                .forward(1.5)
                .addTemporalMarker(2.5, () -> {
                    mtrI.setPower(.7);
                })
                .addTemporalMarker(0, () -> {
                    robot.intakeRight.setPosition(.94);
                    gripper.setPosition(.6);
                })
                .addTemporalMarker(0.3, () -> {
                    robot.intakeLeft.setPosition(.027);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(-35.00, 35.00))
                .turn(Math.toRadians(-90))

                .build();

        TrajectorySequence insertCenter = drive.trajectorySequenceBuilder(RedBackDropCenterTrajs1.end())
                .addTemporalMarker(0, () -> {
                    mtrI.setPower(0);
                })
                .lineTo(new Vector2d(49.00, 35.00))
                .build();

        TrajectorySequence RedBackDropCenterTrajs2 = drive.trajectorySequenceBuilder(insertCenter.end())
                .waitSeconds(1)

                .addTemporalMarker(2.5, () -> {
                    robot.mtrLift.setVelocity(1000);
                })
                .addTemporalMarker(3.1, () -> {
                    robot.mtrLift.setVelocity(0);
                    robot.armSwing.setPosition(1.0);
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence RedBackDropCenterTrajs3 = drive.trajectorySequenceBuilder(RedBackDropCenterTrajs2.end())
                .waitSeconds(.6)
                .addTemporalMarker(.4, () -> {
                    gripper.setPosition(.32);
                })
                .lineTo(new Vector2d(46.00, 35.0))
                .build();


        //is it on the left?
        TrajectorySequence redBackdropLeftTrajs1 = drive.trajectorySequenceBuilder(redBackdropUniversalTraj1.end())
                .lineTo(new Vector2d(-47.00, 46.00))
                .waitSeconds(2)
                .addTemporalMarker(0, () -> {
                    robot.intakeRight.setPosition(.94);
                    gripper.setPosition(.6);
                })
                .addTemporalMarker(0.3, () -> {
                    robot.intakeLeft.setPosition(.027);
                })
                .addTemporalMarker(3.5, () -> {
                    mtrI.setPower(.8);
                    gripper.setPosition(.6);
                })
                .build();

        TrajectorySequence redBackdropLeftTrajs2 = drive.trajectorySequenceBuilder(redBackdropLeftTrajs1.end())
                .addTemporalMarker(0, () -> {
                    mtrI.setPower(0);
                })
                .lineTo(new Vector2d(-36.13, 45.85))
                .lineTo(new Vector2d(-35.00, 35.00))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(50.00, 35.00))
                .build();

        TrajectorySequence redBackdropLeftTrajs3 = drive.trajectorySequenceBuilder(redBackdropLeftTrajs2.end())
                .lineTo(new Vector2d(49, 41.50))
                .addTemporalMarker(0, () -> {
                    robot.mtrLift.setVelocity(1000);
                })
                .addTemporalMarker(.5, () -> {
                    robot.armSwing.setPosition(1.0);
                })
                .addTemporalMarker(.7, () -> {
                    robot.mtrLift.setVelocity(0);
                })
                .addTemporalMarker(2, () -> {
                    gripper.setPosition(.32);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(46, 41.5))
                .build();



        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", bluepropPipeline.getPropPosition());
            telemetry.update();
            auto = bluepropPipeline.getPropPosition();
            if (bluepropPipeline.getPropPosition() == "left"){
                state = 10;
                parkPose = (new Pose2d(46.00, 27, Math.toRadians(180)));
            } else if (bluepropPipeline.getPropPosition() == "right") {
                state = 20;
                parkPose = new Pose2d(46.00, 44, Math.toRadians(180));
            } else {
                state = 30;
                parkPose = (new Pose2d(46.00,35.00, Math.toRadians(180)));
            }

        }

        TrajectorySequence outsidePark = drive.trajectorySequenceBuilder(parkPose)
                .addTemporalMarker(0, () -> {
                    robot.armSwing.setPosition(0);
                })
                .waitSeconds(.5)
                .lineTo(new Vector2d(46.00, 60.25))
                .build();

        TrajectorySequence reset = drive.trajectorySequenceBuilder(outsidePark.end())
                .lineTo(new Vector2d(47, 60.25))
                .addTemporalMarker(0, () -> {
                    robot.mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.mtrLift.setVelocity(1000);
                })
                .addTemporalMarker(0.7, () -> {
                    robot.mtrLift.setVelocity(0);
                })
                .back(1)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        if (state != 10 && state != 30) {
            drive.followTrajectorySequence(redBackdropUniversalTraj1);
        }

        switch (state){
            case (10):
                drive.followTrajectorySequence(redBackdropLeftTrajs1);
                drive.followTrajectorySequence(redBackdropLeftTrajs2);
                drive.followTrajectorySequence(redBackdropLeftTrajs3);
                break;
            case (20):
                drive.followTrajectorySequence(redBackdropRightTrajs1);
                drive.followTrajectorySequence(redBackdropRightTrajs2);
                drive.followTrajectorySequence(insertRight);
                drive.followTrajectorySequence(redBackdropRightTrajs3);
                break;
            case (30):
                drive.followTrajectorySequence(RedBackDropCenterTrajs1);
                drive.followTrajectorySequence(insertCenter);
                drive.followTrajectorySequence(RedBackDropCenterTrajs2);
                drive.followTrajectorySequence(RedBackDropCenterTrajs3);
                break;
        }

        drive.setPoseEstimate(parkPose);
        //if switch == high or whatever, we'll put this in
        drive.followTrajectorySequence(outsidePark);
        //else
        //inside park trajectory

        //drive.setPoseEstimate(parkPose);
        //drive.followTrajectorySequence(reset);

        robot.mtrLift.setVelocity(0);

        while (!isStopRequested() && opModeIsActive()) ;

    }
}
