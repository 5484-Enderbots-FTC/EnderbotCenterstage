package org.firstinspires.ftc.teamcode.PATCHY.AUTOS.PURPLEPIXELONLY;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.bluepropPipeline;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.PATCHY.hardwareCS;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "blue Purple Pixel", group = "Purple Pixel Autos")
public class BluePurplePixel extends LinearOpMode {

    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.bluepropPipeline bluepropPipeline;
    DcMotorEx mtrI;

    private String auto;
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

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", bluepropPipeline.getPropPosition());
            telemetry.update();
            auto = bluepropPipeline.getPropPosition();
        }


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    robot.intakeRight.setPosition(.94);
                    robot.gripper.setPosition(.6);
                })
                .addTemporalMarker(0.3, () -> {
                    robot.intakeLeft.setPosition(.027);
                })
                .forward(20)
                .build();

        //left traj
        TrajectorySequence traj2a = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(65))
                .build();
        TrajectorySequence traj3a = drive.trajectorySequenceBuilder(traj2a.end())
                .forward(3)
                .build();

        //center traj
        TrajectorySequence traj2b = drive.trajectorySequenceBuilder(traj1.end())
                .forward(4)
                .strafeLeft(5)
                .forward(1)
                .build();

        //right traj
        TrajectorySequence traj2c = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(-65))
                .build();
        TrajectorySequence traj3c = drive.trajectorySequenceBuilder(traj2c.end())
                .forward(3)
                .build();

        TrajectorySequence blah = drive.trajectorySequenceBuilder(new Pose2d(22,0,Math.toRadians(0)))
                        .waitSeconds(2)
                .back(10)
                                .build();

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", bluepropPipeline.getPropPosition());
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        auto = bluepropPipeline.getPropPosition();
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

        telemetry.addLine(auto);
        telemetry.update();
        if (isStopRequested()) return;

        robot.intakeRight.setPosition(0.94);
        robot.intakeLeft.setPosition(0.027);

        drive.followTrajectory(traj1);



        if (auto == "left"){
            drive.followTrajectorySequence(traj2a);
            drive.followTrajectorySequence(traj3a);
        } else if (auto == "right"){
            drive.followTrajectorySequence(traj2c);
            drive.followTrajectorySequence(traj3c);
        }else {
            drive.followTrajectorySequence(traj2b);

        }

        mtrI.setPower(.8);
        drive.setPoseEstimate(new Pose2d(22,0,Math.toRadians(0)));
        drive.followTrajectorySequence(blah);
        mtrI.setPower(0);


        while (!isStopRequested() && opModeIsActive()) ;

    }
}
