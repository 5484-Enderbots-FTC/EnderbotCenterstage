package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(name = "Auto")

public class CenterstageAuto extends LinearOpMode {
    public static double DISTANCE = 48; // in
    private DcMotorEx mtrLift = null;
    Servo grab;
    Servo arm;
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 17,18,19 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int firstHeight = 1400;
    int secondHeight = 3100;
    int fullHeight = 4850;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }

                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        mtrLift = hardwareMap.get(DcMotorEx.class, "mtrLift");
        mtrLift.setZeroPowerBehavior(BRAKE);
        mtrLift.setDirection(DcMotor.Direction.REVERSE);
        mtrLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);

        grab = hardwareMap.get(Servo.class, "grab");
        grab.setDirection(Servo.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);


        if (isStopRequested()) return;
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    // This marker runs two seconds into the trajectory
                    arm.setPosition(0.125);
                    // Run your action in here!
                })
                .addTemporalMarker(4, () -> {
                    // This marker runs two seconds into the trajectory
                    grab.setPosition(.92);
                    mtrLift.setTargetPosition(secondHeight);
                    mtrLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    mtrLift.setVelocity(5000);
                    // Run your action in here!
                })
                .addTemporalMarker(6, () -> {
                    // This marker runs two seconds into the trajectory
                    arm.setPosition(0.2);
                    // Run your action in here!
                })
                .addTemporalMarker(8, () -> {
                    // This marker runs two seconds into the trajectory

                    mtrLift.setTargetPosition(0);
                    mtrLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    mtrLift.setVelocity(5000);
                    grab.setPosition(.15);
                    // Run your action in here!
                })
                .forward(DISTANCE + (DISTANCE / 8))
                .strafeRight((DISTANCE / 4) + 2)
                .forward((DISTANCE / 12) - 1.5)
                .waitSeconds(4.5)
                .build();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .back((DISTANCE / 12) - 1.5)
                .strafeLeft((DISTANCE / 4)+1)
                .turn(Math.toRadians(90))
                .forward((DISTANCE / 2) + 5)
                .addTemporalMarker(4, () -> {
                    // This marker runs two seconds into the trajectory
                    arm.setPosition(.3);
                    grab.setPosition(.38);
                    // Run your action in here!
                })
                .addTemporalMarker(6, () -> {
                    // This marker runs two seconds into the trajectory

                    arm.setPosition(.125);
                    // Run your action in here!
                })
                .addTemporalMarker(6.5, () -> {
                    // This marker runs two seconds into the trajectory

                    mtrLift.setTargetPosition(firstHeight);
                    // Run your action in here!
                })
                .waitSeconds(1.5)
                .build();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq2.end())
                .back((DISTANCE / 2) +3)
                .strafeLeft((DISTANCE / 3) -2)
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                })
                .build();
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .addTemporalMarker(1, () -> {
                    // This marker runs two seconds into the trajectory
                    grab.setPosition(.93 );
                    mtrLift.setTargetPosition(secondHeight-150);
                    mtrLift.setVelocity(5000);
                    // Run your action in here!
                })
                .addTemporalMarker(4, () -> {
                    // This marker runs two seconds into the trajectory
                    arm.setPosition(.3);
                    // Run your action in here!
                })
                .addTemporalMarker(6, () -> {
                    // This marker runs two seconds into the trajectory
                    grab.setPosition(.42);
                    mtrLift.setTargetPosition(0);
                    mtrLift.setVelocity(5000);
                    // Run your action in here!
                })
                .waitSeconds(5)
                .build();
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(trajSeq4.end())
                .strafeLeft(DISTANCE / 3)
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(trajSeq5.end())
                .back((DISTANCE / 2)+2)
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(trajSeq5.end())
                .forward((DISTANCE / 2))
                .build();
        /*
        mtrLift.setTargetPosition(0);
        mtrLift.setVelocity(1700);
        arm.setPosition(0.175);
        grab.setPosition(0.15);
        */
        waitForStart();
        drive.followTrajectorySequence(trajSeq1);
        /*
        mtrLift.setTargetPosition(secondHeight);
        arm.setPosition(0.3);
        grab.setPosition(.96);
        mtrLift.setTargetPosition(0);
        grab.setPosition(0.3);

         */

        drive.followTrajectorySequence(trajSeq2);
        drive.followTrajectorySequence(trajSeq3);
        drive.followTrajectorySequence(trajSeq4);
        drive.followTrajectorySequence(trajSeq5);

        if (tagOfInterest.id == 1) {
            drive.followTrajectorySequence(park1);
        } else if (tagOfInterest.id == 3) {
            drive.followTrajectorySequence(park2);
        }
    }



    void tagToTelemetry(AprilTagDetection detection) {

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}