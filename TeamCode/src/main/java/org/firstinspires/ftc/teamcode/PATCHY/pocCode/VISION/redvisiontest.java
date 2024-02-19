package org.firstinspires.ftc.teamcode.PATCHY.pocCode.VISION;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.redpropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Vision Test")
@Disabled
public class redvisiontest extends LinearOpMode {


    private VisionPortal portal;
    private org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.redpropPipeline redpropPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Red Prop Position", redpropPipeline.getPropPosition());
            telemetry.update();
        }
    }

}
