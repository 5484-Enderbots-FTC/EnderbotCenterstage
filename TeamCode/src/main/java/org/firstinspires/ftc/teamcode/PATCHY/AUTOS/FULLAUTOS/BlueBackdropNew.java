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
import org.firstinspires.ftc.teamcode.PATCHY.hardwareCS;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue, close to backdrop", group = "Final Autos")
public class BlueBackdropNew extends LinearOpMode {

    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.bluepropPipeline bluepropPipeline;
    DcMotorEx mtrI;

    Servo gripper;

    private String auto;

    private int state;

    ElapsedTime lifttime = new ElapsedTime();

    Pose2d parkPose;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareCS robot = new hardwareCS();
        robot.inithardware(hardwareMap);

        robot.mtrBL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.mtrBR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.mtrFL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.mtrFR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);

        gripper = hardwareMap.get(Servo.class, "svrGrip");

        gripper.setPosition(.6);
        telemetry.addLine("Gripper should be open.");

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



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12.00, 62.75, Math.toRadians(270.00));
        drive.setPoseEstimate(startPose);

        TrajectorySequence blueBackdropUniversalTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12.00, 36.50))
                .addTemporalMarker(0, () -> {
                    robot.intakeRight.setPosition(.94);
                    gripper.setPosition(.6);
                })
                .addTemporalMarker(0.3, () -> {
                    robot.intakeLeft.setPosition(.027);
                })
                .build();

        //if prop is on the left
        TrajectorySequence blueBackdropLeftTrajs1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12.00, 60.75))
                .lineTo(new Vector2d(20.00, 60.75))
                .lineTo(new Vector2d(20.00, 45.00))
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

        TrajectorySequence blueBackdropLeftTrajs2 = drive.trajectorySequenceBuilder(blueBackdropLeftTrajs1.end())
                .addTemporalMarker(0, () -> {
                    mtrI.setPower(0);
                })
                //to understand this turn motion, please go to learnroadrunner.com and read the 180 turn angle page in the advanced tips section
                .lineTo(new Vector2d(20.00, 50.00))
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence blueBackdropLeftTrajs3 = drive.trajectorySequenceBuilder(blueBackdropLeftTrajs2.end())
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
                .lineTo(new Vector2d(46, 44))
                .build();


        //if prop is in the center
        TrajectorySequence blueBackdropCenterTrajs1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(15.70, 37.5))
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
                .lineTo(new Vector2d(19.00, 42.00))
                .build();
        TrajectorySequence blueBackdropCenterTrajs2 = drive.trajectorySequenceBuilder(blueBackdropCenterTrajs1.end())
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    mtrI.setPower(0);
                })
                .addTemporalMarker(2.5, () -> {
                    robot.mtrLift.setVelocity(1000);
                })
                .addTemporalMarker(3.1, () -> {
                    robot.mtrLift.setVelocity(0);
                    robot.armSwing.setPosition(1.0);
                })
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(49.00, 35.00))

                .build();
        TrajectorySequence blueBackdropCenterTrajs3 = drive.trajectorySequenceBuilder(blueBackdropCenterTrajs2.end())
                .waitSeconds(.6)
                .addTemporalMarker(.4, () -> {
                    gripper.setPosition(.32);
                })
                .lineTo(new Vector2d(46.00, 35.0))
                .build();


        //if the prop is on the right
        TrajectorySequence blueBackdropRightTrajs1 = drive.trajectorySequenceBuilder(blueBackdropUniversalTraj1.end())
                .turn(Math.toRadians(-90))
                .addTemporalMarker(1, () -> {
                    mtrI.setPower(.6);
                    gripper.setPosition(.57);
                })
                .waitSeconds(1.5)
                .build();

        TrajectorySequence blueBackdropRightTrajs2 = drive.trajectorySequenceBuilder(blueBackdropRightTrajs1.end())
                .lineTo(new Vector2d(16.00, 36.5))
                .lineTo(new Vector2d(16.00, 26.5))
                .addTemporalMarker(0, () -> {
                    mtrI.setPower(0);
                    gripper.setPosition(.57);
                })
                .build();

        TrajectorySequence blueBackdropRightTrajs3 = drive.trajectorySequenceBuilder(blueBackdropRightTrajs2.end())
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
                .lineTo(new Vector2d(50.50, 27))
                .waitSeconds(1)
                .lineTo(new Vector2d(46, 27))
                .build();




        //outside parking trajectory
        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", bluepropPipeline.getPropPosition());
            telemetry.update();
            if (bluepropPipeline.getPropPosition() == "left"){
                state = 10;
                parkPose = (new Pose2d(46, 44, Math.toRadians(180)));
            } else if (bluepropPipeline.getPropPosition() == "right") {
                state = 20;
                parkPose = (new Pose2d(46.00, 27, Math.toRadians(180)));
            } else {
                state = 30;
                parkPose = (new Pose2d(46.00,35.00, Math.toRadians(180)));
            }
            sleep(20);



        }

        TrajectorySequence outsidePark = drive.trajectorySequenceBuilder(parkPose)
                .addTemporalMarker(0, () -> {
                    robot.armSwing.setPosition(0);
                })
                .waitSeconds(.5)
                .lineTo(new Vector2d(46.00, 60.25))
                .build();

        //wait until we start
        waitForStart();

        if (isStopRequested()) return;
        telemetry.addData("place", state);

        //gripper.setPosition(.6);
        if (state != 10 && state != 30) {
            drive.followTrajectorySequence(blueBackdropUniversalTraj1);
        }

        switch (state){
            case (10):
                drive.followTrajectorySequence(blueBackdropLeftTrajs1);
                drive.followTrajectorySequence(blueBackdropLeftTrajs2);
                drive.followTrajectorySequence(blueBackdropLeftTrajs3);
                break;
            case (20):
                drive.followTrajectorySequence(blueBackdropRightTrajs1);
                drive.followTrajectorySequence(blueBackdropRightTrajs2);
                drive.followTrajectorySequence(blueBackdropRightTrajs3);
                break;
            case (30):
                drive.followTrajectorySequence(blueBackdropCenterTrajs1);
                drive.followTrajectorySequence(blueBackdropCenterTrajs2);
                drive.followTrajectorySequence(blueBackdropCenterTrajs3);
                break;
        }

        drive.setPoseEstimate(parkPose);
        //if switch == high or whatever, we'll put this in
        drive.followTrajectorySequence(outsidePark);
        //else
        //inside park trajectory

        while (!robot.bottomLimit.isPressed()) {
            robot.mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.armSwing.setPosition(.1);
            lifttime.reset();
            robot.mtrLift.setVelocity(1000);
            if (robot.bottomLimit.isPressed() || lifttime.time() >= .25) {
                robot.mtrLift.setVelocity(0);
                break;
            }
        }

        robot.mtrLift.setVelocity(0);

        while (!isStopRequested() && opModeIsActive()) ;

    }
}
