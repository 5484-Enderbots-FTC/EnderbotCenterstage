package org.firstinspires.ftc.teamcode.PATCHY.AUTOS.FULLAUTOS;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    ElapsedTime lifttime;

    double resolution;
    Pose2d parkPose;
    Servo gripper;

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareCS robot = new hardwareCS();
        robot.inithardware(hardwareMap);

        mtrI = hardwareMap.get(DcMotorEx.class, "mtrI");
        mtrI.setZeroPowerBehavior(BRAKE);
        mtrI.setDirection(DcMotor.Direction.FORWARD);
        mtrI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gripper = hardwareMap.get(Servo.class, "svrGrip");

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
    drive.setPoseEstimate(startPose);




    //one universal trajectory
    TrajectorySequence redBackdropUniversalTraj1 = drive.trajectorySequenceBuilder(startPose)
            .lineTo(new Vector2d(12.00, -36.50))
            .addTemporalMarker(0, () -> {
                robot.intakeRight.setPosition(.94);
                gripper.setPosition(.6);
            })
            .addTemporalMarker(0.3, () -> {
                robot.intakeLeft.setPosition(.027);
            })
            .build();


    //is it on the left?
        TrajectorySequence redBackdropLeftTrajs1 = drive.trajectorySequenceBuilder(redBackdropUniversalTraj1.end())
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
                .build();

        TrajectorySequence redBackdropLeftTrajs2 = drive.trajectorySequenceBuilder(redBackdropLeftTrajs1.end())
                .lineTo(new Vector2d(16.00, -36.5))
                .lineTo(new Vector2d(16.00, -26.5))
                .addTemporalMarker(0, () -> {
                    mtrI.setPower(0);
                    gripper.setPosition(.57);
                })
                .build();

        TrajectorySequence redBackdropLeftTrajs3 = drive.trajectorySequenceBuilder(redBackdropLeftTrajs2.end())
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
                .addTemporalMarker(2.4, () -> {
                    robot.armSwing.setPosition(.8);
                })
                .lineTo(new Vector2d(50.50, -32))
                .waitSeconds(1)
                .lineTo(new Vector2d(46, -32))
                .build();

    //center trajectories
    TrajectorySequence RedBackDropCenterTrajs1 = drive.trajectorySequenceBuilder(startPose)
            .lineTo(new Vector2d(15.70, -37.5))
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
            .lineTo(new Vector2d(19.00, -42.00))
            .build();

    TrajectorySequence RedBackDropCenterTrajs2 = drive.trajectorySequenceBuilder(RedBackDropCenterTrajs1.end())
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
            .turn(Math.toRadians(90))
            .lineTo(new Vector2d(51.00, -41))

            .build();

    TrajectorySequence RedBackDropCenterTrajs3 = drive.trajectorySequenceBuilder(RedBackDropCenterTrajs2.end())
            .waitSeconds(.6)
            .addTemporalMarker(.4, () -> {
                gripper.setPosition(.32);
            })
            .addTemporalMarker(.8, () -> {
                robot.armSwing.setPosition(.8);
            })
            .lineTo(new Vector2d(46.00, -41))
            .build();


    //is it on the right?
    TrajectorySequence redBackdropRightTrajs1 = drive.trajectorySequenceBuilder(startPose)
            .lineTo(new Vector2d(23.50, -45.00))
            .waitSeconds(1.5)
            .addTemporalMarker(0, () -> {
                robot.intakeRight.setPosition(.94);
                gripper.setPosition(.6);
            })
            .addTemporalMarker(0.3, () -> {
                robot.intakeLeft.setPosition(.027);
            })
            .addTemporalMarker(2.5, () -> {
                mtrI.setPower(.8);
                gripper.setPosition(.6);
            })
            .build();

    TrajectorySequence redBackdropRightTrajs2 = drive.trajectorySequenceBuilder(redBackdropRightTrajs1.end())
            //to understand this turn motion, please go to learnroadrunner.com and read the 180 turn angle page in the advanced tips section
            .addTemporalMarker(0, () -> {
                mtrI.setPower(0);
            })
            //to understand this turn motion, please go to learnroadrunner.com and read the 180 turn angle page in the advanced tips section
            .lineTo(new Vector2d(20.00, -50.00))
            .turn(Math.toRadians(90))
            .build();

    TrajectorySequence redBackdropRightTrajs3 = drive.trajectorySequenceBuilder(redBackdropRightTrajs2.end())
            .lineTo(new Vector2d(49, -48))
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
            .addTemporalMarker(2.4, () -> {
                robot.armSwing.setPosition(.8);
            })
            .waitSeconds(1)
            .lineTo(new Vector2d(46, -48))
            .build();


        gripper.setPosition(.6);
        //outside parking trajectory
        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", redpropPipeline.getPropPosition());
            telemetry.update();
            if (redpropPipeline.getPropPosition() == "left"){
                state = 10;
                parkPose = (new Pose2d(46.00, -28.5, Math.toRadians(180)));
            } else if (redpropPipeline.getPropPosition() == "right") {
                state = 20;
                parkPose = new Pose2d(46.00, -45.5, Math.toRadians(180));
            } else {
                state = 30;
                parkPose = (new Pose2d(46.00,-41, Math.toRadians(180)));
            }
            sleep(20);



        }

        //this one is the outside parking pose
        TrajectorySequence outsidePark = drive.trajectorySequenceBuilder(parkPose)
                .lineTo(new Vector2d(46.00, -61.00))
                .addTemporalMarker(0, () -> {
                    robot.armSwing.setPosition(0.1);
                })
                .build();


        //reset thingy

        waitForStart();

        if (isStopRequested()) return;

        gripper.setPosition(.6);
        robot.armSwing.setPosition(0);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

        if (state == 10) {
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
                drive.followTrajectorySequence(redBackdropRightTrajs3);
                break;
            case (30):
                drive.followTrajectorySequence(RedBackDropCenterTrajs1);
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


    }
}
