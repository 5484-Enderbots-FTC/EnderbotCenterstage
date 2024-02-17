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

    private String auto;

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

        auto = redpropPipeline.getPropPosition();
        telemetry.addData("Red Prop Position", redpropPipeline.getPropPosition());
        telemetry.update();

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startPose = new Pose2d(12.00, -62.75, Math.toRadians(90.00));

    TrajectorySequence redBackdropCenterTrajsOutsidePark = drive.trajectorySequenceBuilder(startPose)
            .lineTo(new Vector2d(12.00, -34.50))
            .lineTo(new Vector2d(17.00, -34.50))
            .lineTo(new Vector2d(51.50, -34.50))
            .lineTo(new Vector2d(48.00, -34.50))
            .lineTo(new Vector2d(48.00, -60.00))
            .build();


        waitForStart();
        auto = redpropPipeline.getPropPosition();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(redBackdropCenterTrajsOutsidePark);
    }
}
