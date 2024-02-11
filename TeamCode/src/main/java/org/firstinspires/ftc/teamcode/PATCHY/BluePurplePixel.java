package org.firstinspires.ftc.teamcode.PATCHY;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardwareCS;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "blue Purple Pixel", group = "Purple Pixel Autos")
public class BluePurplePixel extends LinearOpMode {

    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.bluepropPipeline bluepropPipeline;
    DcMotorEx mtrI;

    private String auto;
    @Override
    public void runOpMode() throws InterruptedException {

        hardwareCS robot = new hardwareCS();
        robot.inithardware(hardwareMap);

        mtrI =  hardwareMap.get(DcMotorEx.class, "mtrI");
        mtrI.setZeroPowerBehavior(BRAKE);
        mtrI.setDirection(DcMotor.Direction.FORWARD);
        mtrI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        init_loop();
        {
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
            telemetry.addData("Red Prop Position", bluepropPipeline.getPropPosition());
            telemetry.update();
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .build();

        //left traj
        TrajectorySequence traj2a = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(60))
                .build();

        //center traj
        TrajectorySequence traj2b = drive.trajectorySequenceBuilder(traj1.end())
                .forward(5)
                .strafeLeft(5)
                .build();

        //right traj
        TrajectorySequence traj2c = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(-60))
                .build();

        waitForStart();
        drive.followTrajectorySequence(traj1);

        if (auto == "left"){
            drive.followTrajectorySequence(traj2a);
            mtrI.setPower(.7);
        } else if (auto == "right"){
            drive.followTrajectorySequence(traj2c);
            mtrI.setPower(.7);
        }else {
            drive.followTrajectorySequence(traj2b);
            mtrI.setPower(.7);
        }

    }
}
