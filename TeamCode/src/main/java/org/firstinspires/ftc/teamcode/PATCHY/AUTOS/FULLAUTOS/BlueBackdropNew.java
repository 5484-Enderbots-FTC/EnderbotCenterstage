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

    private String auto;

    private int state;

    ElapsedTime lifttime;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareCS robot = new hardwareCS();
        robot.inithardware(hardwareMap);

        robot.mtrBL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.mtrBR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.mtrFL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.mtrFR.setDirection(DcMotorSimple.Direction.FORWARD);

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
        Pose2d startPose = new Pose2d(12.00, 62.75, Math.toRadians(270.00));

        TrajectorySequence blueBackdropUniversalTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12.00, 34.50))
                .addTemporalMarker(0, () -> {
                    robot.intakeRight.setPosition(.94);
                    robot.gripper.setPosition(.57);
                })
                .addTemporalMarker(0.3, () -> {
                    robot.intakeLeft.setPosition(.027);
                })
                .build();


        //if prop is in the center
        TrajectorySequence blueBackdropCenterTrajs1 = drive.trajectorySequenceBuilder(blueBackdropUniversalTraj1.end())
                .lineTo(new Vector2d(17.00, 34.50))
                .addTemporalMarker(2, () -> {
                    mtrI.setPower(.6);
                })
                .waitSeconds(2)
                .build();
        TrajectorySequence blueBackdropCenterTrajs2 = drive.trajectorySequenceBuilder(blueBackdropCenterTrajs1.end())
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    mtrI.setPower(0);
                })
                .addTemporalMarker(2.5, () -> {
                    robot.mtrLift.setVelocity(1000);
                })
                .addTemporalMarker(3.25, () -> {
                    robot.mtrLift.setVelocity(0);
                })
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(51.50, 34.50))

                .build();
        TrajectorySequence blueBackdropCenterTrajs3 = drive.trajectorySequenceBuilder(blueBackdropCenterTrajs2.end())
                .waitSeconds(2)
                .addTemporalMarker(1, () -> {
                    robot.gripper.setPosition(.32);
                })
                .lineTo(new Vector2d(48.00, 34.50))
                .build();



        //outside parking trajectory
        TrajectorySequence outsidePark = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(48.00, 60.00))
                .build();


        waitForStart();
        if (bluepropPipeline.getPropPosition() == "left"){
            state = 10;
        } else if (bluepropPipeline.getPropPosition() == "right") {
            state = 20;
        } else {
            state = 30;
        }

        if (isStopRequested()) return;

        drive.followTrajectorySequence(blueBackdropUniversalTraj1);

        switch (state){
            case (10):

                break;
            case (20):

                break;
            case (30):
                drive.followTrajectorySequence(blueBackdropCenterTrajs1);
                drive.followTrajectorySequence(blueBackdropCenterTrajs2);
                drive.followTrajectorySequence(blueBackdropCenterTrajs3);
                break;
        }

        //if switch == high or whatever, we'll put this in
        drive.followTrajectorySequence(outsidePark);
        //else
        //inside park trajectory

        while (!robot.bottomLimit.isPressed()) {
            robot.mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
            lifttime.reset();
            robot.mtrLift.setVelocity(1000);
            if (robot.bottomLimit.isPressed() || lifttime.time() >= .75 || robot.bottomLimit.isPressed() && lifttime.time() >= .75) {
                robot.mtrLift.setVelocity(0);
                break;
            }
        }

    }
}
