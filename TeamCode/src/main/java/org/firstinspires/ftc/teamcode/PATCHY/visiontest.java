package org.firstinspires.ftc.teamcode.PATCHY;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

    @Autonomous(name = "Vision Test")
    public class visiontest extends LinearOpMode {


        private VisionPortal portal;
        private org.firstinspires.ftc.teamcode.PATCHY.redpropPipeline redpropPipeline;

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
                    telemetry.addData("Red Prop Position", redpropPipeline.getPropPosition());
                    telemetry.update();

            }

        }
