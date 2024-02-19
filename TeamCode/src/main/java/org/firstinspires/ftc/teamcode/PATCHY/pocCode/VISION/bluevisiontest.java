package org.firstinspires.ftc.teamcode.PATCHY.pocCode.VISION;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.bluepropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue Vision Test")
@Disabled
    public class bluevisiontest extends LinearOpMode {


        private VisionPortal portal;
        private org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.bluepropPipeline bluepropPipeline;

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

                waitForStart();
                while (opModeIsActive()) {
                    telemetry.addData("Blue Prop Position", bluepropPipeline.getPropPosition());
                    telemetry.update();
                }
            }

        }
